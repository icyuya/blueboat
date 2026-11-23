import io
import json
import unittest
from contextlib import redirect_stdout
from unittest.mock import Mock, patch

import coex_target_receiver as receiver


SOURCE = ("127.0.0.1", 16001)
SESSION = "a81c9314-9719-49ec-b843-b7cd1ec7beca"
OTHER_SESSION = "5d340c46-e753-4c29-87df-02f4762763ab"
WALL_NOW = 1_700_000_000


def packet(**overrides):
    data = {
        "type": "target",
        "lat": 26.0000449,
        "lon": 127.0,
        "confidence": 0.95,
        "sent_at": WALL_NOW,
        "session_id": SESSION,
        "seq": 0,
    }
    return json.dumps({**data, **overrides}).encode("utf-8")


class CoexTargetGateTests(unittest.TestCase):
    def setUp(self):
        self.gate = receiver.CoexTargetGate(SOURCE, SESSION)

    def accept(self, *, at=100, wall=WALL_NOW, source=SOURCE, **overrides):
        return self.gate.accept(
            packet(**overrides), source, wall_now=wall, monotonic_now=at,
        )

    def test_consecutive_targets_accept_gaps_and_refresh_timeout(self):
        for seq, at in ((0, 100), (1, 102), (4, 104), (5, 106)):
            with self.subTest(seq=seq):
                self.assertEqual(self.accept(seq=seq, at=at)["seq"], seq)
        self.assertIsNone(self.gate.check_timeout(108.99))

    def test_duplicate_and_out_of_order_targets_are_rejected(self):
        self.accept(seq=10)
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.accept(seq=10, at=101)
        with self.assertRaisesRegex(ValueError, "Out-of-order"):
            self.accept(seq=9, at=102)
        self.assertEqual(self.accept(seq=11, at=102.5)["seq"], 11)

    def test_replays_do_not_extend_timeout(self):
        self.accept(seq=10)
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.accept(seq=10, at=102)
        with self.assertRaisesRegex(ValueError, "Out-of-order"):
            self.accept(seq=9, at=102.5)
        self.assertEqual(self.gate.check_timeout(103), "COEX target timeout")

    def test_wrong_ip_or_port_cannot_latch_or_advance_sequence(self):
        self.accept()
        for source in (("127.0.0.2", SOURCE[1]), (SOURCE[0], 16002)):
            with self.subTest(source=source):
                with self.assertRaisesRegex(ValueError, "Unexpected COEX source"):
                    self.accept(seq=100, source=source, session_id=OTHER_SESSION, at=101)
        self.assertIsNone(self.gate.latch_reason)
        self.assertEqual(self.accept(seq=1, at=101)["seq"], 1)

    def test_existing_payload_rejections_do_not_advance_sequence_or_latch(self):
        self.accept()
        for overrides, message in (
            ({"sent_at": WALL_NOW - 10, "session_id": OTHER_SESSION}, "Expired timestamp"),
            ({"sent_at": WALL_NOW + 10}, "Expired timestamp"),
            ({"confidence": 0.79}, "Low confidence"),
            ({"lat": 91}, "Invalid latitude"),
            ({"lon": 181}, "Invalid longitude"),
        ):
            with self.subTest(overrides=overrides):
                with self.assertRaisesRegex(ValueError, message):
                    self.accept(seq=100, at=101, **overrides)
        self.assertIsNone(self.gate.latch_reason)
        self.assertEqual(self.accept(seq=1, at=102)["seq"], 1)

    def test_session_and_sequence_fields_are_required(self):
        for field in ("session_id", "seq"):
            data = json.loads(packet())
            del data[field]
            with self.subTest(field=field):
                with self.assertRaisesRegex(ValueError, f"Invalid {field}"):
                    self.gate.accept(
                        json.dumps(data).encode("utf-8"), SOURCE,
                        wall_now=WALL_NOW, monotonic_now=100,
                    )

    def test_rejects_invalid_session_and_sequence_values(self):
        for session_id in (None, 1, "", "not-a-uuid", SESSION.upper()):
            with self.subTest(session_id=session_id):
                with self.assertRaisesRegex(ValueError, "Invalid session_id"):
                    self.accept(session_id=session_id)
        for seq in (None, True, "0", 0.0, -1, receiver.MAX_SEQUENCE + 1):
            with self.subTest(seq=seq):
                with self.assertRaisesRegex(ValueError, "Invalid seq"):
                    self.accept(seq=seq)
        self.assertEqual(self.accept()["seq"], 0)

    def test_uint32_sequence_does_not_wrap(self):
        self.assertEqual(self.accept(seq=receiver.MAX_SEQUENCE)["seq"], receiver.MAX_SEQUENCE)
        with self.assertRaisesRegex(ValueError, "Out-of-order"):
            self.accept(seq=0, at=101)

    def test_session_change_latches_and_neither_session_can_resume(self):
        self.accept(seq=20)
        with self.assertRaisesRegex(ValueError, "LATCHED: Unexpected COEX session"):
            self.accept(seq=0, session_id=OTHER_SESSION, at=101)
        for session_id in (SESSION, OTHER_SESSION):
            with self.subTest(session_id=session_id):
                with self.assertRaisesRegex(ValueError, "LATCHED: Unexpected COEX session"):
                    self.accept(seq=21, session_id=session_id, at=102)
        self.assertEqual(self.gate.check_timeout(200), "Unexpected COEX session")

    def test_first_packet_cannot_choose_session(self):
        with self.assertRaisesRegex(ValueError, "LATCHED: Unexpected COEX session"):
            self.accept(session_id=OTHER_SESSION)
        with self.assertRaisesRegex(ValueError, "LATCHED: Unexpected COEX session"):
            self.accept()

    def test_packet_after_timeout_cannot_resume_even_without_poll(self):
        self.accept()
        with self.assertRaisesRegex(ValueError, "LATCHED: COEX target timeout"):
            self.accept(seq=1, at=103, wall=WALL_NOW + 3, sent_at=WALL_NOW + 3)
        with self.assertRaisesRegex(ValueError, "LATCHED: COEX target timeout"):
            self.accept(seq=0, session_id=OTHER_SESSION, at=104)

    def test_clock_skew_is_separate_from_monotonic_timeout(self):
        self.assertIsNone(self.gate.check_timeout(100))
        self.accept(seq=0, at=100, wall=WALL_NOW - 4)
        self.accept(seq=1, at=102, wall=WALL_NOW + 4)
        self.assertIsNone(self.gate.check_timeout(104.99))
        self.assertEqual(self.gate.check_timeout(105), "COEX target timeout")


class CoexMonitorTests(unittest.TestCase):
    def test_mocked_receiver_stops_on_timeout_without_processing_queued_packet(self):
        datagrams = Mock()
        datagrams.recvfrom.side_effect = [
            (packet(seq=0), SOURCE),
            (packet(seq=0), SOURCE),
            (packet(seq=1), SOURCE),
            receiver.socket.timeout,
            (packet(seq=2), SOURCE),
        ]
        gate = receiver.CoexTargetGate(SOURCE, SESSION)
        with (
            patch.object(receiver.time, "monotonic", side_effect=[100, 101, 102, 105]),
            patch.object(receiver.time, "time", return_value=WALL_NOW),
            redirect_stdout(io.StringIO()) as output,
        ):
            receiver.monitor_targets(datagrams, gate)

        self.assertEqual(datagrams.recvfrom.call_count, 4)
        self.assertEqual(output.getvalue().count("TARGET ACCEPTED:"), 2)
        self.assertIn("TARGET REJECTED: Duplicate", output.getvalue())
        self.assertIn("COEX LATCHED: COEX target timeout", output.getvalue())


if __name__ == "__main__":
    unittest.main()
