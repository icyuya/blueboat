"""Deterministic, receive-only evaluation of one fixed-target trial."""
import json
import math
from enum import StrEnum
from types import SimpleNamespace

from pymavlink import mavutil

from blueboat_ekf_health import EkfReport, ekf_block_reasons
from blueboat_navigator import STANDOFF_M, destination, distance_bearing
from blueboat_navigator_live_v2 import (
    BOAT_COMPID, BOAT_SYSID, GPS_TIMEOUT, HEARTBEAT_TIMEOUT, MAX_HDOP,
    MAX_TARGET_DISTANCE, MIN_SATELLITES, SAFE_MODES,
)


class State(StrEnum):
    HEALTH = "waiting_health"
    PERMISSION = "waiting_permission"
    VEHICLE = "waiting_vehicle"
    APPROACH = "approach"
    REQUEST = "stop_requested"
    OBSERVE = "observing_stop"
    STOPPED = "recorded_stop"
    ABORT = "stop_unconfirmed"


def valid_point(lat, lon):
    return (type(lat) in (int, float) and type(lon) in (int, float)
            and -90 <= lat <= 90 and -180 <= lon <= 180)


# At most one independent observation per type and reception timestamp.
FIELDS = {
    "HEARTBEAT": ("type", "autopilot", "base_mode", "custom_mode"),
    "GPS_RAW_INT": ("fix_type", "satellites_visible", "eph"),
    "EKF_STATUS_REPORT": ("flags",),
    "GLOBAL_POSITION_INT": ("time_boot_ms", "lat", "lon", "vx", "vy"),
}


class Trial:
    def __init__(self, trial_id, lat, lon):
        if not isinstance(trial_id, str) or not trial_id.strip() or not valid_point(lat, lon):
            raise ValueError("Expected trial ID and valid decimal degrees")
        self.trial_id = trial_id
        self._target = (lat, lon)
        self.state = State.HEALTH
        self.start = self.approach = None
        self.now = self.heartbeat = self.gps = self.ekf = self.position = None
        self.mode = self.armed = None
        self.request = self.reason = None
        self.window = []
        self.rows = []
        self._pending_at = None
        self._messages = {}
        self._actions = set()

    @property
    def target(self):
        return self._target

    def health(self):
        hb = self.heartbeat is not None and self.now - self.heartbeat <= HEARTBEAT_TIMEOUT
        gps = self.gps is not None and self.now - self.gps[0] <= GPS_TIMEOUT
        if gps:
            _, fix, satellites, eph = self.gps
            gps = fix >= 3 and MIN_SATELLITES <= satellites < 255 and 0 < eph <= MAX_HDOP * 100
        pos = self.position
        fresh = bool(pos and self.now - pos["at"] <= GPS_TIMEOUT
                     and valid_point(*pos["point"]) and pos["point"] != (0, 0))
        ekf = ekf_block_reasons(self.ekf, monotonic_now=self.now)
        return dict(heartbeat=hb, gnss=gps, position=fresh, ekf_reasons=ekf)

    def record(self, event):
        point = self.position and self.position["point"]
        distances = (None, None)
        if point and valid_point(*point) and self.approach:
            distances = tuple(distance_bearing(*point, *p)[0] for p in (self.approach, self.target))
        self.rows.append(dict(trial_id=self.trial_id, at=self.now, event=event, state=self.state,
                              health=self.health(), heartbeat_at=self.heartbeat, gps=self.gps,
                              ekf=self.ekf and dict(at=self.ekf.received_at, flags=self.ekf.flags),
                              position=self.position, target=self.target,
                              start=self.start, approach=self.approach, approach_m=distances[0],
                              target_m=distances[1], mode=self.mode, armed=self.armed,
                              stop_request=self.request, evidence=list(self.window), reason=self.reason))

    def jsonl(self):
        return "".join(json.dumps(row, allow_nan=False) + "\n" for row in self.rows)

    def transition(self, state, reason=None):
        self.state, self.reason = state, reason
        self.record("transition")

    def advance(self, at, manual=False):
        if self.state in (State.ABORT, State.STOPPED):
            return False
        if not math.isfinite(at) or (self.now is not None and at < self.now):
            self.transition(State.ABORT, "invalid event time")
            return False
        first_event = self.now is None
        self.now = at
        if first_event:
            self.record("start")
        if manual:
            self.transition(State.ABORT, "manual intervention")
            return False
        h = self.health()
        if self.heartbeat is not None and not h["heartbeat"]:
            self.transition(State.ABORT, "heartbeat timeout")
        elif self.state != State.HEALTH and (not h["gnss"] or not h["position"] or h["ekf_reasons"]):
            self.transition(State.ABORT, "navigation health lost")
        return self.state != State.ABORT

    def _queue(self, at):
        if self.state == State.ABORT:
            return False
        if (type(at) not in (int, float) or not math.isfinite(at) or at < 0
                or (self.now is not None and at <= self.now)
                or (self._pending_at is not None and at < self._pending_at)):
            self._messages.clear()
            self._actions.clear()
            self._pending_at = None
            self.transition(State.ABORT, "invalid or already closed event time")
            return False
        if self._pending_at is not None and at > self._pending_at:
            self.close_timestamp()
        if self.state in (State.ABORT, State.STOPPED):
            return False
        self._pending_at = at
        return True

    def close_timestamp(self):
        """Seal the pending group after the producer has delivered all its events."""
        at = self._pending_at
        if at is None:
            return
        actions, messages = self._actions, self._messages
        self._actions, self._messages, self._pending_at = set(), {}, None
        if not self.advance(at, manual="manual" in actions):
            return
        if "invalid" in actions:
            self.transition(State.ABORT, "missing or conflicting telemetry")
            return
        for kind in FIELDS:
            if kind in messages:
                fields = dict(zip(FIELDS[kind], messages[kind]))
                if kind == "HEARTBEAT":
                    fields["mode"] = messages[kind][-1]
                self._observe(kind, SimpleNamespace(**fields), at)
                if self.state == State.ABORT:
                    return
        if "permit" in actions and self.state == State.PERMISSION:
            self.transition(State.VEHICLE)
        if "poll" in actions:
            self.record("poll")

    def poll(self, at):
        if self._queue(at):
            self._actions.add("poll")

    def permit(self, at):
        if self._queue(at):
            self._actions.add("permit")

    def manual_intervention(self, at):
        if self._queue(at):
            self._actions.add("manual")

    def finish(self, at):
        if at != self.now or self._pending_at is not None:
            self._queue(at)
        self.close_timestamp()
        if self.state not in (State.ABORT, State.STOPPED):
            self.transition(State.ABORT, "end of input before confirmed stop")

    def observe(self, msg, at, *, manual=False):
        if not self._queue(at):
            return
        if manual:
            self._actions.add("manual")
        if (msg.get_srcSystem(), msg.get_srcComponent()) != (BOAT_SYSID, BOAT_COMPID):
            return
        kind = msg.get_type()
        if kind not in FIELDS:
            return
        values = tuple(getattr(msg, field, None) for field in FIELDS[kind])
        required = values[:-2] if kind == "GLOBAL_POSITION_INT" else values
        if any(type(value) is not int for value in required):
            self._actions.add("invalid")
            return
        if kind == "GLOBAL_POSITION_INT" and not 0 <= values[0] <= 0xffffffff:
            self._actions.add("invalid")
            return
        if kind == "HEARTBEAT" and values[:2] != (
                mavutil.mavlink.MAV_TYPE_SURFACE_BOAT, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA):
            return
        if kind == "HEARTBEAT":
            values += (mavutil.mode_string_v10(msg),)
        if kind in self._messages and values != self._messages[kind]:
            self._actions.add("invalid")
        self._messages[kind] = values

    def _observe(self, kind, msg, at):
        if kind == "HEARTBEAT":
            mav = mavutil.mavlink
            previous = self.mode
            self.heartbeat = at
            self.mode = msg.mode
            self.armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
            if self.state in (State.HEALTH, State.PERMISSION):
                allowed = not self.armed and self.mode in SAFE_MODES and previous in (None, self.mode)
            elif self.state == State.VEHICLE:
                allowed = self.mode == "GUIDED" or (self.mode == previous and self.mode in SAFE_MODES)
                if allowed and self.mode == "GUIDED" and self.armed:
                    self.transition(State.APPROACH)
            else:
                allowed = self.mode == "GUIDED" and self.armed and previous != "HOLD"
                if self.state in (State.REQUEST, State.OBSERVE) and self.mode == "HOLD":
                    allowed = True
            if not allowed:
                self.transition(State.ABORT, "unexpected vehicle state or manual override")
                return
        elif kind == "GPS_RAW_INT":
            self.gps = (at, msg.fix_type, msg.satellites_visible, msg.eph)
        elif kind == "EKF_STATUS_REPORT":
            self.ekf = EkfReport(at, msg.flags)
        elif kind == "GLOBAL_POSITION_INT":
            boot = msg.time_boot_ms
            if self.position and boot == self.position["boot_ms"]:
                return
            if self.position and boot < self.position["boot_ms"]:
                self.transition(State.ABORT, "position clock regressed")
                return
            velocity = (getattr(msg, "vx", None), getattr(msg, "vy", None))
            speed = None
            if all(type(v) is int and -32768 <= v < 32767 for v in velocity):
                speed = math.hypot(*velocity) / 100
            self.position = dict(at=at, boot_ms=boot, point=(msg.lat / 1e7, msg.lon / 1e7),
                                 speed_mps=speed)
        else:
            return
        if not self.advance(at):
            return
        h = self.health()
        if self.state == State.HEALTH and h["heartbeat"] and h["gnss"] and h["position"] and not h["ekf_reasons"]:
            self.start = self.position["point"]
            distance, bearing = distance_bearing(*self.start, *self.target)
            if not STANDOFF_M < distance <= MAX_TARGET_DISTANCE:
                self.transition(State.ABORT, "target outside permitted distance")
                return
            self.approach = destination(*self.start, bearing, distance - STANDOFF_M)
            self.transition(State.PERMISSION)
        if kind == "GLOBAL_POSITION_INT":
            self.evaluate_position()
        self.record(kind)

    def evaluate_position(self):
        p = self.position
        if self.state == State.APPROACH and distance_bearing(*p["point"], *self.approach)[0] <= 1:
            self.request = dict(at=self.now, reason="within approach radius")
            self.transition(State.REQUEST)
        elif self.state in (State.REQUEST, State.OBSERVE) and self.now > self.request["at"]:
            if self.state == State.REQUEST:
                self.transition(State.OBSERVE)
            if p["speed_mps"] is None or p["speed_mps"] > 0.2:
                self.window.clear()
                return
            if self.window:
                last = self.window[-1]
                if (not 0 < p["at"] - last["at"] <= 1.5
                        or p["boot_ms"] - last["boot_ms"] > 1500
                        or any(distance_bearing(*s["point"], *p["point"])[0] > 0.5 for s in self.window)):
                    self.window.clear()
            self.window.append(p)
            first = self.window[0]
            if p["at"] - first["at"] >= 2 and p["boot_ms"] - first["boot_ms"] >= 2000:
                self.transition(State.STOPPED, "speed and displacement criteria met")
