import unittest

from analyze_blueboat_tlog_gaps import BOAT, Packet, inside, match_gaps, ordered_gaps, sequence_steps


def packet(timestamp, seq=0):
    return Packet(timestamp, BOAT, "HEARTBEAT", seq)


class TlogGapAnalysisTests(unittest.TestCase):
    def test_strict_three_second_gap_boundary(self):
        start, middle, end = map(packet, (0, 3_000_000, 6_000_001))
        self.assertEqual(ordered_gaps([start, middle, end]), [(middle, end)])

    def test_open_interval_excludes_equal_timestamps_at_both_bounds(self):
        packets = list(map(packet, (0, 1, 1, 2, 2, 3, 3)))
        self.assertEqual(inside(packets, packet(1), packet(3)), [packet(2), packet(2)])

    def test_sequence_wrap_and_ambiguous_steps(self):
        packets = [packet(index, seq) for index, seq in enumerate((254, 255, 0, 0, 2, 1))]
        self.assertEqual(sequence_steps(packets), (2, 1, 1, 1))

    def test_overlap_matches_are_one_to_one(self):
        first = [(packet(1), packet(8)), (packet(9), packet(20)), (packet(50), packet(60))]
        second = [(packet(2), packet(7)), (packet(7), packet(19)), (packet(60), packet(70))]
        self.assertEqual(match_gaps(first, second), [(0, 0, 5), (1, 1, 10)])
