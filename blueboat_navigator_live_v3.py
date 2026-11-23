import logging
import math
from dataclasses import dataclass

from pymavlink import mavutil

from blueboat_ekf_health import EKF_MAX_AGE_S, EkfHealthMonitor
from blueboat_navigator import STANDOFF_M, destination, distance_bearing
from blueboat_navigator_live_v2 import (
    BOAT_COMPID,
    BOAT_SYSID,
    GPS_TIMEOUT,
    HEARTBEAT_TIMEOUT,
    MAX_HDOP,
    MAX_TARGET_DISTANCE,
    MIN_SATELLITES,
    SAFE_MODES,
)
from coex_target_receiver import CoexTargetGate


LOGGER = logging.getLogger(__name__)
MAV = mavutil.mavlink


@dataclass(frozen=True)
class DryRunWaypoint:
    lat: float
    lon: float


class DryRunNavigator:
    def __init__(self, *, coex_source, coex_session_id):
        self._coex = CoexTargetGate(coex_source, coex_session_id)
        self._ekf = EkfHealthMonitor(system_id=BOAT_SYSID, component_id=BOAT_COMPID)
        self._heartbeat_at = None
        self._mode = None
        self._gps = None
        self._position = None
        self._ekf_received_at = None
        self._navigation_was_healthy = False
        self._latch_reason = None
        self._last_reasons = None
        self._last_time = None

    @property
    def latch_reason(self):
        return self._latch_reason

    def _latch(self, reason):
        if self._latch_reason is None:
            self._latch_reason = reason
            LOGGER.warning("DRY RUN SAFETY LATCHED: %s; vehicle stop not implemented", reason)

    def manual_intervention(self):
        self._latch("Manual intervention reported")

    def _gnss_reason(self, now):
        if self._gps is None or now - self._gps[0] > GPS_TIMEOUT:
            return "GPS data unavailable or stale"

        _, fix, satellites, eph = self._gps
        if fix < 3:
            return f"No 3D GPS Fix (fix={fix})"
        if satellites == 255 or satellites < MIN_SATELLITES:
            return "Insufficient GPS satellites"
        if not 0 < eph <= MAX_HDOP * 100:
            return "GPS HDOP too high, invalid or unknown"
        if self._position is None or now - self._position[0] > GPS_TIMEOUT:
            return "Position data unavailable or stale"

        _, lat, lon = self._position
        if (lat, lon) == (0, 0) or not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return "Position invalid"
        return None

    def poll(self, *, monotonic_now):
        if not math.isfinite(monotonic_now) or (
            self._last_time is not None and monotonic_now < self._last_time
        ):
            self._latch("Invalid monotonic event time")
            return (f"SAFETY LATCHED: {self._latch_reason}",)
        self._last_time = monotonic_now

        if self._heartbeat_at is not None and monotonic_now - self._heartbeat_at > HEARTBEAT_TIMEOUT:
            self._latch("Heartbeat timeout")
        coex_reason = self._coex.check_timeout(monotonic_now)
        if coex_reason is not None:
            self._latch(coex_reason)

        nav_reasons = self._ekf.block_reasons(
            monotonic_now=monotonic_now, gnss_reason=self._gnss_reason(monotonic_now),
        )
        waiting_for_heartbeat = self._heartbeat_at is None
        if nav_reasons and self._navigation_was_healthy:
            self._latch("Position health lost: " + "; ".join(nav_reasons))
        elif not nav_reasons and not waiting_for_heartbeat and self._latch_reason is None:
            self._navigation_was_healthy = True

        reasons = ("No ArduRover heartbeat",) if waiting_for_heartbeat else ()
        reasons += nav_reasons
        if self._latch_reason is not None:
            reasons = (f"SAFETY LATCHED: {self._latch_reason}", *reasons)
        if reasons and reasons != self._last_reasons:
            LOGGER.warning("DRY RUN GUIDED BLOCKED: %s", "; ".join(reasons))
        self._last_reasons = reasons
        return reasons

    def observe_telemetry(self, msg, *, received_at):
        self.poll(monotonic_now=received_at)
        if not math.isfinite(received_at) or received_at != self._last_time:
            return
        if (msg.get_srcSystem(), msg.get_srcComponent()) != (BOAT_SYSID, BOAT_COMPID):
            return

        kind = msg.get_type()
        if kind == "HEARTBEAT":
            if msg.type != MAV.MAV_TYPE_SURFACE_BOAT or msg.autopilot != MAV.MAV_AUTOPILOT_ARDUPILOTMEGA:
                return
            mode = mavutil.mode_string_v10(msg)
            self._heartbeat_at = received_at
            if msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED:
                self._latch("Vehicle armed during DRY RUN")
            if mode not in SAFE_MODES:
                self._latch(f"Unexpected mode: {mode}")
            if self._mode is not None and mode != self._mode:
                self._latch(f"Mode changed: {self._mode} -> {mode}")
            self._mode = mode
        elif kind == "GPS_RAW_INT":
            self._gps = (received_at, msg.fix_type, msg.satellites_visible, msg.eph)
        elif kind == "GLOBAL_POSITION_INT":
            self._position = (received_at, msg.lat / 1e7, msg.lon / 1e7)
        elif self._ekf.observe(msg, received_at=received_at):
            if self._ekf_received_at is not None:
                interval = received_at - self._ekf_received_at
                LOGGER.info("EKF receive interval: %.3f s (timeout %.3f s)", interval, EKF_MAX_AGE_S)
            self._ekf_received_at = received_at

        self.poll(monotonic_now=received_at)

    def accept_target(self, raw, source, *, wall_now, monotonic_now):
        reasons = self.poll(monotonic_now=monotonic_now)
        if self._latch_reason is not None:
            return None

        try:
            target = self._coex.accept(raw, source, wall_now=wall_now, monotonic_now=monotonic_now)
        except ValueError as exc:
            LOGGER.warning("DRY RUN TARGET REJECTED: %s", exc)
            if self._coex.latch_reason is not None:
                self._latch(self._coex.latch_reason)
            return None
        if reasons:
            return None

        _, lat, lon = self._position
        distance, bearing = distance_bearing(lat, lon, target["lat"], target["lon"])
        if distance > MAX_TARGET_DISTANCE:
            LOGGER.warning("DRY RUN GUIDED BLOCKED: Target too far (%.2f m)", distance)
            return None
        if distance <= STANDOFF_M:
            LOGGER.warning("DRY RUN GUIDED BLOCKED: Inside standoff distance (%.2f m)", distance)
            return None

        goal = DryRunWaypoint(*destination(lat, lon, bearing, distance - STANDOFF_M))
        LOGGER.info("DRY RUN waypoint (not sent): %.7f, %.7f", goal.lat, goal.lon)
        return goal


if __name__ == "__main__":
    print("DRY RUN only: inject offline telemetry/targets; no connection, transmission or vehicle stop.")
