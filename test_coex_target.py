import io
import json
import unittest
from contextlib import redirect_stdout
from unittest.mock import call, patch

import blueboat_navigator_live_v2 as live
from blueboat_navigator import (
    MAX_TARGET_PACKET_BYTES,
    TARGET_RECV_BUFFER_BYTES,
    parse_target_packet,
    validate,
)


NOW = 1_700_000_000.0
TARGET = {
    "type": "target",
    "lat": 26.0000449,
    "lon": 127.0,
    "confidence": 0.95,
    "sent_at": NOW,
}


def packet(**overrides):
    return json.dumps({**TARGET, **overrides}).encode("utf-8")


class TargetValidationTests(unittest.TestCase):
    def test_valid_packet(self):
        self.assertEqual(parse_target_packet(packet(), now=NOW), TARGET)

    def test_inclusive_boundaries(self):
        for latitude, longitude, confidence, offset in (
            (-90, -180, 0.8, -5),
            (90, 180, 1, 5),
        ):
            with self.subTest(offset=offset):
                parse_target_packet(
                    packet(
                        lat=latitude,
                        lon=longitude,
                        confidence=confidence,
                        sent_at=NOW + offset,
                    ),
                    now=NOW,
                )

    def test_rejects_old_and_future_timestamps(self):
        for offset in (-10, -5.001, 5.001, 10):
            with self.subTest(offset=offset):
                with self.assertRaisesRegex(ValueError, "Expired timestamp"):
                    parse_target_packet(packet(sent_at=NOW + offset), now=NOW)

    def test_rejects_confidence_outside_allowed_range(self):
        for confidence in (0.7999, 1.0001):
            with self.subTest(confidence=confidence):
                with self.assertRaisesRegex(ValueError, "Low confidence"):
                    parse_target_packet(packet(confidence=confidence), now=NOW)

    def test_rejects_coordinates_outside_allowed_ranges(self):
        for key, invalid, message in (
            ("lat", (-90.001, 90.001), "Invalid latitude"),
            ("lon", (-180.001, 180.001), "Invalid longitude"),
        ):
            for value in invalid:
                with self.subTest(key=key, value=value):
                    with self.assertRaisesRegex(ValueError, message):
                        parse_target_packet(packet(**{key: value}), now=NOW)

    def test_rejects_missing_or_invalid_numeric_fields(self):
        for key in ("lat", "lon", "confidence", "sent_at"):
            with self.subTest(key=key, missing=True):
                incomplete = TARGET.copy()
                del incomplete[key]
                with self.assertRaisesRegex(ValueError, f"^Invalid {key}$"):
                    validate(incomplete, now=NOW)
            for value in (
                True, None, "26", float("nan"), float("inf"),
                float("-inf"), 10**400,
            ):
                with self.subTest(key=key, value=value):
                    with self.assertRaisesRegex(ValueError, f"^Invalid {key}$"):
                        validate({**TARGET, key: value}, now=NOW)

    def test_rejects_wrong_message_shape_or_type(self):
        for raw in (b"[]", b"null", b"42"):
            with self.subTest(raw=raw):
                with self.assertRaisesRegex(ValueError, "Invalid JSON object"):
                    parse_target_packet(raw, now=NOW)
        with self.assertRaisesRegex(ValueError, "Invalid message type"):
            parse_target_packet(packet(type="status"), now=NOW)

    def test_rejects_malformed_json_and_utf8(self):
        with self.assertRaises(json.JSONDecodeError):
            parse_target_packet(b'{"type":', now=NOW)
        with self.assertRaises(UnicodeDecodeError):
            parse_target_packet(b"\xff", now=NOW)

    def test_rejects_nonstandard_json_constants_and_float_overflow(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "Invalid JSON constant"):
                    parse_target_packet(packet(lat=value), now=NOW)
        raw = packet(lat="overflow").replace(b'"overflow"', b"1e309")
        with self.assertRaisesRegex(ValueError, "^Invalid lat$"):
            parse_target_packet(raw, now=NOW)

    def test_rejects_duplicate_keys(self):
        raw = packet()[:-1] + b', "lat": 27}'
        with self.assertRaisesRegex(ValueError, "Duplicate JSON key"):
            parse_target_packet(raw, now=NOW)

    def test_handles_decoder_recursion_error(self):
        with patch.object(json, "loads", side_effect=RecursionError):
            with self.assertRaisesRegex(ValueError, "nested too deeply"):
                parse_target_packet(packet(), now=NOW)

    def test_packet_size_limit(self):
        raw = packet().ljust(MAX_TARGET_PACKET_BYTES)
        self.assertEqual(parse_target_packet(raw, now=NOW), TARGET)
        with self.assertRaisesRegex(ValueError, "Target packet too large"):
            parse_target_packet(raw + b" ", now=NOW)


class LiveTargetIngressTests(unittest.TestCase):
    def test_v2_rejects_bad_packets_and_continues_without_heartbeat(self):
        sender = ("127.0.0.1", 16001)
        with (
            patch.object(live.mavutil, "mavlink_connection") as connect,
            patch.object(live.socket, "socket") as open_socket,
            patch.object(live.time, "time", return_value=NOW),
            patch.object(live, "create_guided") as create_guided,
            redirect_stdout(io.StringIO()) as output,
        ):
            master = connect.return_value
            master.recv_match.return_value = None
            receiver = open_socket.return_value.__enter__.return_value
            receiver.recvfrom.side_effect = [
                (b'{"type":', sender),
                (packet(lat=10**400), sender),
                (packet().ljust(MAX_TARGET_PACKET_BYTES + 1), sender),
                (packet(), sender),
                KeyboardInterrupt,
            ]
            live.main()

        log = output.getvalue()
        self.assertEqual(log.count("TARGET REJECTED:"), 3)
        self.assertEqual(log.count("TARGET RECEIVED"), 1)
        self.assertIn("GUIDED BLOCKED: No ArduRover heartbeat", log)
        self.assertEqual(
            receiver.recvfrom.call_args_list,
            [call(TARGET_RECV_BUFFER_BYTES)] * 5,
        )
        create_guided.assert_not_called()
        master.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
