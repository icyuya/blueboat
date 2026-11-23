import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from blueboat_navigator_v4 import State, Trial
from pymavlink import mavutil


class Message(SimpleNamespace):
    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return self.source[0]

    def get_srcComponent(self):
        return self.source[1]


def hb(mode=4, armed=False):
    return Message(kind="HEARTBEAT", source=(1, 1), type=11, autopilot=3,
                   custom_mode=mode, base_mode=1 | (128 if armed else 0))


def sample(trial, t, point, *, speed=0, boot=None, source=(1, 1)):
    trial.observe(Message(kind="GLOBAL_POSITION_INT", source=source,
                          time_boot_ms=round(t * 1000) if boot is None else boot,
                          lat=round(point[0] * 1e7), lon=round(point[1] * 1e7),
                          vx=speed, vy=0), t)


def health(trial, t, fix=3, flags=23):
    trial.observe(Message(kind="GPS_RAW_INT", source=(1, 1), fix_type=fix,
                          satellites_visible=10, eph=100), t)
    trial.observe(Message(kind="EKF_STATUS_REPORT", source=(1, 1), flags=flags), t)


def ready():
    trial = Trial("synthetic", 26.0001, 127)
    trial.observe(hb(), 0)
    health(trial, 0)
    sample(trial, 0, (26, 127))
    return trial


def arriving():
    trial = ready()
    trial.permit(0)
    trial.observe(hb(15, True), 0.1)
    sample(trial, 0.2, trial.approach)
    return trial


def slow(trial, t, **kwargs):
    trial.observe(hb(4, True), t)
    health(trial, t)
    sample(trial, t, trial.approach, **kwargs)


class OfflineTrialTests(unittest.TestCase):
    def test_full_trial_determinism_and_no_io(self):
        def run():
            trial = arriving()
            approach = trial.approach
            for t in (1, 2, 3):
                slow(trial, t)
            trial.close_timestamp()
            self.assertEqual(trial.approach, approach)
            self.assertEqual(trial.state, State.STOPPED)
            self.assertEqual(trial.request["at"], 0.2)
            self.assertEqual(len(trial.window), 3)
            rows = list(map(json.loads, trial.jsonl().splitlines()))
            self.assertAlmostEqual(rows[-1]["target_m"], 3, delta=0.01)
            self.assertLess(rows[-1]["approach_m"], 0.01)
            states = [r["state"] for r in rows if r["event"] == "transition"]
            self.assertEqual(states, [State.PERMISSION, State.VEHICLE, State.APPROACH,
                                      State.REQUEST, State.OBSERVE, State.STOPPED])
            return trial.jsonl()
        with patch("socket.socket", side_effect=AssertionError), patch.object(
            mavutil, "mavlink_connection", side_effect=AssertionError
        ):
            self.assertEqual(run(), run())

    def test_invalid_targets_and_immutable_target(self):
        for point in [(91, 0), (0, -181), (False, 0), (float("nan"), 0), (1, "1")]:
            with self.assertRaises(ValueError):
                Trial("test", *point)
        with self.assertRaises(AttributeError):
            ready().target = (1, 1)

    def test_permissions_and_intervention(self):
        for mode, armed in [(4, True), (15, False), (0, False)]:
            trial = ready()
            trial.observe(hb(mode, armed), 1)
            trial.close_timestamp()
            self.assertEqual(trial.state, State.ABORT)
        trial = arriving()
        slow(trial, 1)
        slow(trial, 2)
        trial.observe(hb(), 3, manual=True)
        slow(trial, 3)
        trial.close_timestamp()
        self.assertEqual(trial.reason, "manual intervention")

    def test_health_loss_precedes_completion(self):
        for fix, flags in [(1, 23), (3, 135)]:
            trial = arriving()
            slow(trial, 1)
            slow(trial, 2)
            health(trial, 3, fix, flags)
            sample(trial, 3, trial.approach)
            trial.close_timestamp()
            self.assertEqual(trial.state, State.ABORT)

    def test_three_second_boundaries_and_late_heartbeat(self):
        trial = ready()
        trial.poll(3)
        trial.close_timestamp()
        self.assertTrue(trial.health()["heartbeat"])
        self.assertTrue(trial.health()["position"])
        self.assertTrue(trial.health()["gnss"])
        self.assertEqual(trial.health()["ekf_reasons"], ("EKF_STATUS_REPORT expired",))
        trial = Trial("gap", 26, 127)
        trial.observe(hb(), 0)
        trial.observe(hb(), 3.001)
        trial.close_timestamp()
        self.assertEqual(trial.reason, "heartbeat timeout")

    def test_stale_position_cannot_be_repaired_by_late_sample(self):
        trial = ready()
        trial.observe(hb(), 2)
        health(trial, 2)
        trial.poll(3)
        trial.close_timestamp()
        self.assertEqual(trial.state, State.PERMISSION)
        sample(trial, 3.001, trial.start)
        trial.close_timestamp()
        self.assertEqual(trial.state, State.ABORT)
        self.assertFalse(trial.health()["position"])

    def test_foreign_heartbeat_cannot_start_approach(self):
        trial = ready()
        trial.permit(0)
        message = hb(15, True)
        message.source = (2, 1)
        trial.observe(message, 1)
        trial.close_timestamp()
        self.assertEqual(trial.state, State.VEHICLE)

    def test_stop_needs_distinct_fresh_stationary_samples(self):
        for defect in ("duplicate", "speed", "displacement", "source", "missing", "gap"):
            with self.subTest(defect=defect):
                trial = arriving()
                slow(trial, 1)
                for t in (2, 3):
                    trial.observe(hb(4), t)
                    health(trial, t)
                    if defect == "speed":
                        sample(trial, t, trial.approach, speed=50)
                    elif defect == "displacement":
                        point = (trial.approach[0] + (t - 1) * 0.00001, 127)
                        sample(trial, t, point)
                    elif defect == "duplicate":
                        sample(trial, t, trial.approach, boot=1000)
                    elif defect == "source":
                        sample(trial, t, trial.approach, source=(1, 2))
                    elif defect == "gap" and t == 3:
                        sample(trial, t, trial.approach)
                trial.finish(3)
                self.assertEqual(trial.state, State.ABORT)

    def test_november_19_measured_intervals_with_synthetic_heartbeats(self):
        # First heartbeat gap from each local replay CSV, rounded to milliseconds.
        for start, end in [(3.142, 7.141), (1.906, 5.904)]:
            trial = Trial("gap-projection", 26, 127)
            trial.observe(hb(), start)
            trial.observe(hb(), end)
            trial.close_timestamp()
            self.assertEqual(trial.reason, "heartbeat timeout")

class LocalRecordingTests(unittest.TestCase):
    def test_november_19_recordings(self):
        import os
        from contextlib import closing
        from pathlib import Path

        directory = os.environ.get("BLUEBOAT_NOV19_LOG_DIR")
        if not directory:
            self.skipTest("Set BLUEBOAT_NOV19_LOG_DIR to replay the two local recordings")
        for name, expected in [("14-38-40", 6.2), ("14-38-41-1", 5.0)]:
            path = Path(directory) / f"2026-11-19 {name}.tlog"
            self.assertTrue(path.is_file())
            trial = Trial(name, 26.0001, 127)
            origin = None
            next_tick = 0
            with closing(mavutil.mavlogfile(str(path), write=False)) as log:
                for msg in iter(log.recv_msg, None):
                    if msg.get_type() == "BAD_DATA":
                        continue
                    stamp = round(msg._timestamp * 1_000_000)
                    if origin is None:
                        origin = stamp
                    elapsed = stamp - origin
                    while next_tick <= elapsed:
                        trial.poll(next_tick / 1_000_000)
                        next_tick += 100_000
                    trial.observe(msg, elapsed / 1_000_000)
            self.assertEqual(trial.reason, "heartbeat timeout")
            self.assertAlmostEqual(trial.now, expected)
            self.assertIsNone(trial.request)


class TimestampConflictTests(unittest.TestCase):
    def check_orders(self, setup, left, right, expected, reason=None):
        logs = []
        for events in ((left, right), (right, left)):
            trial = setup()
            for event in events:
                event(trial)
            trial.close_timestamp()
            self.assertEqual(trial.state, expected)
            if reason:
                self.assertEqual(trial.reason, reason)
            if expected == State.ABORT:
                self.assertFalse(any(row["state"] == State.STOPPED for row in trial.rows))
                trial.observe(hb(), 10)
                self.assertEqual(trial.state, State.ABORT)
            logs.append(trial.jsonl())
        self.assertEqual(*logs)

    @staticmethod
    def approaching():
        trial = ready()
        trial.permit(0)
        trial.observe(hb(15, True), 0.1)
        trial.close_timestamp()
        return trial

    @staticmethod
    def nearly_stopped():
        trial = arriving()
        slow(trial, 1)
        slow(trial, 2)
        trial.close_timestamp()
        return trial

    def test_arrival_and_manual(self):
        self.check_orders(self.approaching, lambda t: sample(t, 1, t.approach),
                          lambda t: t.manual_intervention(1), State.ABORT, "manual intervention")

    def test_arrival_and_bad_ekf(self):
        self.check_orders(self.approaching, lambda t: sample(t, 1, t.approach),
                          lambda t: health(t, 1, flags=135), State.ABORT, "navigation health lost")

    def test_stop_and_bad_gnss(self):
        self.check_orders(self.nearly_stopped, lambda t: sample(t, 3, t.approach),
                          lambda t: health(t, 3, fix=1), State.ABORT, "navigation health lost")

    def test_permit_cannot_authorize_simultaneous_arm_or_mode_change(self):
        for mode, armed in [(4, True), (15, True), (15, False)]:
            self.check_orders(ready, lambda t: t.permit(1),
                              lambda t: t.observe(hb(mode, armed), 1), State.ABORT)

    def test_late_refresh_and_deadline(self):
        self.check_orders(ready, lambda t: t.observe(hb(), 4), lambda t: t.poll(4),
                          State.ABORT, "heartbeat timeout")

    def test_identical_samples_and_conflicting_duplicates(self):
        trial = arriving()
        slow(trial, 1)
        sample(trial, 1, trial.approach)
        trial.close_timestamp()
        self.assertEqual(len(trial.window), 1)
        self.check_orders(self.nearly_stopped, lambda t: sample(t, 3, t.approach),
                          lambda t: sample(t, 3, t.approach, speed=50), State.ABORT)

    def test_invalid_time_and_missing_fields(self):
        for invalid in (None, float("nan"), -1, 0.5):
            trial = ready()
            trial.poll(1)
            trial.poll(invalid)
            self.assertEqual(trial.state, State.ABORT)
        trial = ready()
        trial.observe(Message(kind="GLOBAL_POSITION_INT", source=(1, 1), lat=0), 1)
        trial.close_timestamp()
        self.assertEqual(trial.state, State.ABORT)

    def test_sealed_timestamp_cannot_be_reopened(self):
        trial = self.nearly_stopped()
        slow(trial, 3)
        self.assertEqual(trial.state, State.OBSERVE)
        trial.close_timestamp()
        self.assertEqual(trial.state, State.STOPPED)
        trial.manual_intervention(3)
        self.assertEqual(trial.state, State.ABORT)
