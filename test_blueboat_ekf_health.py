import unittest
from unittest.mock import Mock

from pymavlink.dialects.v20 import ardupilotmega as mav

from blueboat_ekf_health import EKF_MAX_AGE_S, EkfHealthMonitor, EkfReport, ekf_block_reasons


RECEIVED_AT = 100.0
HEALTHY_FLAGS = (
    mav.EKF_ATTITUDE | mav.EKF_VELOCITY_HORIZ | mav.EKF_VELOCITY_VERT | mav.EKF_POS_HORIZ_ABS
)


def message(flags=HEALTHY_FLAGS, *, system_id=1, component_id=1, kind="EKF_STATUS_REPORT"):
    packet = Mock(spec=["get_type", "get_srcSystem", "get_srcComponent"])
    packet.get_type.return_value = kind
    packet.get_srcSystem.return_value = system_id
    packet.get_srcComponent.return_value = component_id
    packet.flags = flags
    return packet


class EkfDecisionTests(unittest.TestCase):
    def reasons(self, flags, *, age=0.0):
        return ekf_block_reasons(EkfReport(RECEIVED_AT, flags), monotonic_now=RECEIVED_AT + age)

    def test_healthy_fresh_report(self):
        self.assertEqual(self.reasons(HEALTHY_FLAGS, age=EKF_MAX_AGE_S - 0.01), ())

    def test_predicted_or_relative_position_is_not_absolute_position(self):
        flags = mav.EKF_ATTITUDE | mav.EKF_POS_HORIZ_REL | mav.EKF_PRED_POS_HORIZ_ABS
        self.assertEqual(self.reasons(flags), ("EKF_POS_HORIZ_ABS not set",))

    def test_constant_position_blocks_even_with_absolute_position(self):
        self.assertEqual(
            self.reasons(HEALTHY_FLAGS | mav.EKF_CONST_POS_MODE), ("EKF_CONST_POS_MODE set",),
        )

    def test_abnormal_flags_block_independently(self):
        for bit, reason in (
            (mav.EKF_UNINITIALIZED, "EKF_UNINITIALIZED set"),
            (mav.EKF_GPS_GLITCHING, "EKF_GPS_GLITCHING set"),
        ):
            with self.subTest(bit=bit):
                self.assertEqual(self.reasons(HEALTHY_FLAGS | bit), (reason,))

    def test_report_not_received(self):
        self.assertEqual(
            ekf_block_reasons(None, monotonic_now=RECEIVED_AT), ("EKF_STATUS_REPORT not received",),
        )

    def test_report_expires_at_boundary(self):
        for age in (EKF_MAX_AGE_S, EKF_MAX_AGE_S + 1):
            with self.subTest(age=age):
                self.assertEqual(self.reasons(HEALTHY_FLAGS, age=age), ("EKF_STATUS_REPORT expired",))

    def test_future_or_nonfinite_receive_time(self):
        for age in (-0.1, float("nan"), float("inf")):
            with self.subTest(age=age):
                self.assertEqual(self.reasons(HEALTHY_FLAGS, age=age), ("Invalid EKF reception time",))

    def test_malformed_flags(self):
        for flags in (None, True, -1, 65536, 16.0, "16"):
            with self.subTest(flags=flags):
                self.assertEqual(self.reasons(flags), ("Invalid EKF_STATUS_REPORT flags",))


class EkfMonitorTests(unittest.TestCase):
    def setUp(self):
        self.monitor = EkfHealthMonitor(system_id=1, component_id=1)

    def check(self, *, gnss_reason=None, now=RECEIVED_AT):
        return self.monitor.block_reasons(monotonic_now=now, gnss_reason=gnss_reason)

    def test_observed_indoor_state_blocks_with_gnss_and_ekf_reasons(self):
        flags = (HEALTHY_FLAGS & ~mav.EKF_POS_HORIZ_ABS) | mav.EKF_CONST_POS_MODE
        self.assertTrue(self.monitor.observe(message(flags), received_at=RECEIVED_AT))
        with self.assertLogs("blueboat_ekf_health", "WARNING") as logs:
            result = self.check(gnss_reason="No 3D GPS Fix (fix=1)")
            self.assertEqual(self.check(gnss_reason="No 3D GPS Fix (fix=1)"), result)
        self.assertEqual(result, (
            "No 3D GPS Fix (fix=1)", "EKF_CONST_POS_MODE set", "EKF_POS_HORIZ_ABS not set",
        ))
        self.assertEqual(len(logs.output), 1)
        self.assertIn("GUIDED BLOCKED:", logs.output[0])
        for reason in result:
            self.assertIn(reason, logs.output[0])

    def test_healthy_ekf_preserves_existing_gnss_blocks(self):
        self.monitor.observe(message(), received_at=RECEIVED_AT)
        for reason in (
            "No 3D GPS Fix (fix=1)", "Insufficient GPS satellites",
            "GPS HDOP too high or unknown", "Position data unavailable",
        ):
            with self.subTest(reason=reason), self.assertLogs("blueboat_ekf_health", "WARNING"):
                self.assertEqual(self.check(gnss_reason=reason), (reason,))
        self.assertEqual(self.check(), ())

    def test_wrong_message_or_source_cannot_refresh_last_sample(self):
        self.monitor.observe(message(), received_at=RECEIVED_AT)
        for incoming in (message(kind="HEARTBEAT"), message(system_id=2), message(component_id=2)):
            self.assertFalse(self.monitor.observe(incoming, received_at=RECEIVED_AT + 2))
        with self.assertLogs("blueboat_ekf_health", "WARNING"):
            self.assertEqual(self.check(now=RECEIVED_AT + EKF_MAX_AGE_S), ("EKF_STATUS_REPORT expired",))

    def test_latest_bad_report_replaces_prior_healthy_report(self):
        self.monitor.observe(message(), received_at=RECEIVED_AT)
        self.assertEqual(self.check(), ())
        invalid = message()
        del invalid.flags
        self.monitor.observe(invalid, received_at=RECEIVED_AT + 1)
        with self.assertLogs("blueboat_ekf_health", "WARNING"):
            self.assertEqual(self.check(now=RECEIVED_AT + 1), ("Invalid EKF_STATUS_REPORT flags",))

    def test_variances_logged_without_thresholds(self):
        incoming = message()
        incoming.velocity_variance = 0.125
        incoming.pos_horiz_variance = 99.0
        with self.assertLogs("blueboat_ekf_health", "DEBUG") as logs:
            self.monitor.observe(incoming, received_at=RECEIVED_AT)
        for text in ("'velocity_variance': 0.125", "'pos_horiz_variance': 99.0", "'airspeed_variance': None"):
            self.assertIn(text, logs.output[0])
        self.assertEqual(self.check(), ())
