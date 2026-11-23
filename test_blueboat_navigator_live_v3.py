import json
import unittest
from unittest.mock import Mock

from pymavlink.dialects.v20 import ardupilotmega as ekf_flags

from blueboat_navigator import STANDOFF_M, distance_bearing
from blueboat_navigator_live_v3 import MAV, DryRunNavigator


SOURCE = ("127.0.0.1", 16001)
SESSION = "a81c9314-9719-49ec-b843-b7cd1ec7beca"
NEW_SESSION = "5d340c46-e753-4c29-87df-02f4762763ab"
WALL = 1_700_000_000
START = 100.0
BOAT_LAT = 26.0
BOAT_LON = 127.0
TARGET_LAT = 26.0001
HEALTHY_FLAGS = 7 | ekf_flags.EKF_POS_HORIZ_ABS


class DryRunIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.navigator = DryRunNavigator(coex_source=SOURCE, coex_session_id=SESSION)
        logs = self.assertLogs("blueboat_navigator_live_v3", level="INFO")
        self.logs = logs.__enter__()
        self.addCleanup(logs.__exit__, None, None, None)
        ekf_logs = self.assertLogs("blueboat_ekf_health", level="WARNING")
        ekf_logs.__enter__()
        self.addCleanup(ekf_logs.__exit__, None, None, None)

    def telemetry(self, kind, at=START, *, system_id=1, component_id=1, **fields):
        msg = Mock(**fields)
        msg.get_type.return_value = kind
        msg.get_srcSystem.return_value = system_id
        msg.get_srcComponent.return_value = component_id
        self.navigator.observe_telemetry(msg, received_at=at)

    def heartbeat(self, at=START, *, mode=4, armed=False, **source):
        base_mode = MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if armed:
            base_mode |= MAV.MAV_MODE_FLAG_SAFETY_ARMED
        self.telemetry(
            "HEARTBEAT", at, type=MAV.MAV_TYPE_SURFACE_BOAT,
            autopilot=MAV.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=base_mode, custom_mode=mode, **source,
        )

    def gps(self, at=START, *, fix=3, satellites=10, eph=100):
        self.telemetry("GPS_RAW_INT", at, fix_type=fix, satellites_visible=satellites, eph=eph)

    def position(self, at=START):
        self.telemetry("GLOBAL_POSITION_INT", at, lat=260000000, lon=1270000000)

    def ekf(self, at=START, *, flags=HEALTHY_FLAGS, **source):
        self.telemetry("EKF_STATUS_REPORT", at, flags=flags, **source)

    def good_telemetry(self, at=START):
        self.heartbeat(at)
        self.gps(at)
        self.position(at)
        self.ekf(at)

    def target(self, at=START, *, seq=0, source=SOURCE, legacy=False, **overrides):
        now = WALL + at - START
        data = {
            "type": "target", "lat": TARGET_LAT, "lon": BOAT_LON,
            "confidence": 0.95, "sent_at": now, **overrides,
        }
        if not legacy:
            data.setdefault("session_id", SESSION)
            data["seq"] = seq
        return self.navigator.accept_target(
            json.dumps(data).encode("utf-8"), source, wall_now=now, monotonic_now=at,
        )

    def test_healthy_telemetry_generates_only_waypoint(self):
        self.good_telemetry()
        for seq in (0, 1, 3):
            waypoint = self.target(seq=seq)
            distance, _ = distance_bearing(waypoint.lat, waypoint.lon, TARGET_LAT, BOAT_LON)
            self.assertAlmostEqual(distance, STANDOFF_M, places=5)
        self.assertIsNone(self.navigator.latch_reason)

    def test_observed_indoor_state_waits_without_latching_or_reusing_target(self):
        self.heartbeat()
        self.gps(fix=1)
        self.position()
        self.ekf(flags=135)
        self.assertIsNone(self.target())
        self.assertIsNone(self.navigator.latch_reason)
        log = "\n".join(self.logs.output)
        for reason in ("No 3D GPS Fix", "EKF_CONST_POS_MODE set", "EKF_POS_HORIZ_ABS not set"):
            self.assertIn(reason, log)
        self.good_telemetry(101)
        self.assertIsNone(self.target(101))
        self.assertIsNotNone(self.target(101, seq=1))

    def test_gnss_loss_after_healthy_latches_without_waiting_for_target(self):
        self.good_telemetry()
        self.gps(101, fix=1)
        self.assertIn("Position health lost: No 3D GPS Fix", self.navigator.latch_reason)
        self.good_telemetry(102)
        self.assertIsNone(self.target(102))

    def test_ekf_loss_then_recovery_in_same_batch_stays_latched(self):
        self.good_telemetry()
        self.ekf(101, flags=HEALTHY_FLAGS | ekf_flags.EKF_GPS_GLITCHING)
        self.ekf(101)
        self.assertIn("EKF_GPS_GLITCHING set", self.navigator.latch_reason)
        self.assertIsNone(self.target(101))

    def test_ekf_interval_is_logged_and_timeout_not_relaxed(self):
        self.good_telemetry()
        self.ekf(101.43)
        self.heartbeat(103)
        self.gps(103)
        self.position(103)
        self.ekf(104, system_id=2)
        self.assertIsNotNone(self.target(104))
        self.assertIsNone(self.target(104.44, seq=1))
        self.assertIn("EKF_STATUS_REPORT expired", self.navigator.latch_reason)
        self.ekf(104.5)
        self.assertIn("EKF receive interval: 1.430 s (timeout 3.000 s)", "\n".join(self.logs.output))
        self.assertIn("EKF receive interval: 3.070 s (timeout 3.000 s)", "\n".join(self.logs.output))
        self.assertIsNone(self.target(104.5, seq=2))

    def test_gps_input_staleness_is_independent(self):
        self.good_telemetry()
        self.heartbeat(102)
        self.position(102)
        self.ekf(102)
        self.navigator.poll(monotonic_now=103.1)
        self.assertIn("GPS data unavailable or stale", self.navigator.latch_reason)

    def test_position_staleness_is_independent(self):
        self.good_telemetry()
        self.heartbeat(102)
        self.gps(102)
        self.ekf(102)
        self.navigator.poll(monotonic_now=103.1)
        self.assertIn("Position data unavailable or stale", self.navigator.latch_reason)

    def test_satellite_and_hdop_checks_remain_required(self):
        self.heartbeat()
        self.position()
        self.ekf()
        for satellites, eph, reason in (
            (7, 100, "Insufficient GPS satellites"),
            (255, 100, "Insufficient GPS satellites"),
            (8, 201, "GPS HDOP"),
            (8, 65535, "GPS HDOP"),
            (8, 0, "GPS HDOP"),
        ):
            with self.subTest(satellites=satellites, eph=eph):
                self.gps(satellites=satellites, eph=eph)
                self.assertIn(reason, self.navigator.poll(monotonic_now=START)[0])
                self.assertIsNone(self.navigator.latch_reason)
        self.gps(satellites=8, eph=200)
        self.assertIsNotNone(self.target())

    def test_heartbeat_gap_is_detected_before_a_late_heartbeat_is_stored(self):
        self.good_telemetry()
        self.gps(102)
        self.position(102)
        self.ekf(102)
        self.heartbeat(103.1)
        self.assertEqual(self.navigator.latch_reason, "Heartbeat timeout")
        self.assertIsNone(self.target(103.2))

    def test_coex_timeout_cannot_be_cleared_by_old_or_new_session(self):
        self.good_telemetry()
        self.assertIsNotNone(self.target())
        self.good_telemetry(102)
        self.assertIsNone(self.target(103, seq=1))
        self.assertEqual(self.navigator.latch_reason, "COEX target timeout")
        self.assertIsNone(self.target(103.1, session_id=NEW_SESSION))

    def test_session_change_latches_and_old_session_cannot_resume(self):
        self.good_telemetry()
        self.assertIsNotNone(self.target())
        self.assertIsNone(self.target(101, session_id=NEW_SESSION))
        self.assertEqual(self.navigator.latch_reason, "Unexpected COEX session")
        self.assertIsNone(self.target(101, seq=1))

    def test_legacy_foreign_stale_and_disordered_coex_targets_are_rejected(self):
        self.good_telemetry()
        self.assertIsNone(self.target(legacy=True))
        self.assertIsNone(self.target(source=(SOURCE[0], 16002)))
        self.assertIsNone(self.target(sent_at=WALL - 10))
        self.assertIsNotNone(self.target(seq=2))
        self.assertIsNone(self.target(seq=2))
        self.assertIsNone(self.target(seq=1))
        self.assertIsNotNone(self.target(seq=3))
        self.assertIsNone(self.navigator.latch_reason)

    def test_switch_from_hold_to_manual_latches_even_though_both_modes_are_allowed(self):
        self.good_telemetry()
        self.heartbeat(101, mode=0)
        self.heartbeat(102)
        self.assertEqual(self.navigator.latch_reason, "Mode changed: HOLD -> MANUAL")
        self.assertIsNone(self.target(102))

    def test_explicit_manual_intervention_blocks_preview(self):
        self.good_telemetry()
        self.navigator.manual_intervention()
        self.assertEqual(self.navigator.latch_reason, "Manual intervention reported")
        self.assertIsNone(self.target())

    def test_armed_heartbeat_latches_even_at_startup(self):
        self.heartbeat(armed=True)
        self.good_telemetry(101)
        self.assertEqual(self.navigator.latch_reason, "Vehicle armed during DRY RUN")
        self.assertIsNone(self.target(101))

    def test_unexpected_initial_mode_latches(self):
        self.heartbeat(mode=15)
        self.assertEqual(self.navigator.latch_reason, "Unexpected mode: GUIDED")
        self.assertIsNone(self.target())

    def test_distance_limits_are_preserved(self):
        self.good_telemetry()
        self.assertIsNone(self.target(lat=BOAT_LAT))
        self.assertIsNone(self.target(seq=1, lat=BOAT_LAT + 0.002))
        log = "\n".join(self.logs.output)
        self.assertIn("Inside standoff distance", log)
        self.assertIn("Target too far", log)
