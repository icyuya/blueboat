import csv
import io
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from pymavlink.dialects.v20 import ardupilotmega as mav

from replay_blueboat_tlog import CSV_FIELDS, percentile, read_tlog, replay, report_rows


ORIGIN = 1_700_000_000


def packet(kind, elapsed, *, system=1, component=1, **fields):
    message = Mock(_timestamp=ORIGIN + elapsed, **fields)
    message.get_type.return_value = kind
    message.get_srcSystem.return_value = system
    message.get_srcComponent.return_value = component
    return message


def heartbeat(elapsed, **source):
    return packet(
        "HEARTBEAT", elapsed, type=mav.MAV_TYPE_SURFACE_BOAT,
        autopilot=mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode=4, **source,
    )


class ReplayTests(unittest.TestCase):
    def setUp(self):
        for logger in ("blueboat_navigator_live_v3.LOGGER", "blueboat_ekf_health.LOGGER"):
            self.enterContext(patch(f"{logger}.disabled", True))

    def test_poll_detects_latch_between_packets_and_keeps_it(self):
        with patch("replay_blueboat_tlog.DryRunNavigator.accept_target") as accept:
            result = replay([
                heartbeat(0), packet("GPS_RAW_INT", 0, fix_type=1, satellites_visible=0, eph=9999),
                packet("EKF_STATUS_REPORT", 0, flags=135),
                heartbeat(2, component=2), heartbeat(2.5, system=2), heartbeat(5),
                packet("EKF_STATUS_REPORT", 5, flags=135), heartbeat(6),
            ])
        accept.assert_not_called()
        self.assertEqual(result.counts["HEARTBEAT"], 3)
        self.assertEqual([event for event in result.events if event[0] == "latch"], [
            ("latch", 3_100_000, "Heartbeat timeout"),
        ])
        self.assertTrue(any("No 3D GPS Fix" in detail and "EKF_CONST_POS_MODE" in detail
                            for _, _, detail in result.events))
        self.assertTrue(any("EKF_STATUS_REPORT expired" in detail for _, _, detail in result.events))
        self.assertIn("SAFETY LATCHED", result.events[-1][2])

    def test_unrelated_messages_define_recording_bounds(self):
        result = replay([packet("ATTITUDE", 0), heartbeat(1), packet("ATTITUDE", 5)])
        self.assertEqual(result.arrivals["HEARTBEAT"], [1_000_000])
        self.assertIn(("latch", 4_100_000, "Heartbeat timeout"), result.events)
        self.assertEqual(result.end_us, 5_000_000)

    def test_interval_statistics_use_strictly_over_three_and_csv_schema(self):
        result = replay([heartbeat(elapsed) for elapsed in (0, 1, 4, 9)])
        rows = list(report_rows(result))
        stats = next(row for row in rows if row["event"] == "interval_stats" and row["kind"] == "HEARTBEAT")
        self.assertEqual((stats["median_s"], stats["max_s"], stats["over_3s"]), (3, 5, 1))
        self.assertAlmostEqual(stats["p95_s"], 4.8)
        self.assertAlmostEqual(stats["p99_s"], 4.96)
        self.assertIn({"event": "gap_over_3s", "kind": "HEARTBEAT", "elapsed_s": 9,
                       "gap_start_s": 4, "gap_s": 5}, rows)
        output = io.StringIO()
        writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
        self.assertEqual(len(list(csv.DictReader(io.StringIO(output.getvalue())))), len(rows))

    def test_backward_timestamps_not_reordered(self):
        with self.assertRaisesRegex(ValueError, "move backwards"):
            replay([heartbeat(0), heartbeat(2), heartbeat(1)])

    def test_no_matching_packets_and_missing_timestamp_are_errors(self):
        with self.assertRaisesRegex(ValueError, "No matching"):
            replay([heartbeat(0, component=2)])
        message = heartbeat(0)
        del message._timestamp
        with self.assertRaisesRegex(ValueError, "invalid tlog receive timestamp"):
            replay([message])

    def test_empty_and_single_interval_percentiles(self):
        self.assertIsNone(percentile([], 99))
        self.assertEqual(percentile([1.5], 99), 1.5)

    def test_binary_tlog_read_uses_record_header_not_vehicle_time(self):
        encoder = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        first = mav.MAVLink_heartbeat_message(
            mav.MAV_TYPE_SURFACE_BOAT, mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 3,
        )
        second = mav.MAVLink_gps_raw_int_message(123, 1, 0, 0, 0, 9999, 0, 0, 0, 0)
        records = [(0, first), (3_521_000, second)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.tlog"
            path.write_bytes(b"".join(
                struct.pack(">Q", ORIGIN * 1_000_000 + offset) + message.pack(encoder)
                for offset, message in records
            ))
            result = read_tlog(path)
        self.assertEqual(result.end_us, 3_521_000)
        self.assertEqual(result.counts["GPS_RAW_INT"], 1)
        self.assertIn(("latch", 3_100_000, "Heartbeat timeout"), result.events)
