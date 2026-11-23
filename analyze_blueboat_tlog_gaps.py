import argparse
from collections import Counter
from contextlib import closing
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from itertools import pairwise
from pathlib import Path

from pymavlink import mavutil


BOAT = (1, 1)
GCS = (255, 190)
CORE = ("GPS_RAW_INT", "GLOBAL_POSITION_INT", "EKF_STATUS_REPORT")
LIMIT_US = 3_000_000
JST = timezone(timedelta(hours=9))


@dataclass(frozen=True)
class Packet:
    timestamp: int
    source: tuple
    kind: str
    seq: int
    heartbeat: bytes | None = None


def read_tlog(path):
    if path.suffix.lower() != ".tlog" or not path.is_file():
        raise ValueError(f"Not a local .tlog file: {path}")
    packets = []
    bad_data = 0
    mav = mavutil.mavlink
    with closing(mavutil.mavlogfile(str(path), write=False, notimestamps=False)) as reader:
        for message in iter(reader.recv_msg, None):
            kind = message.get_type()
            if kind == "BAD_DATA":
                bad_data += 1
                continue
            source = (message.get_srcSystem(), message.get_srcComponent())
            heartbeat = None
            if (source == BOAT and kind == "HEARTBEAT"
                and message.type == mav.MAV_TYPE_SURFACE_BOAT
                and message.autopilot == mav.MAV_AUTOPILOT_ARDUPILOTMEGA):
                heartbeat = bytes(message.get_msgbuf())
            packets.append(Packet(round(message._timestamp * 1_000_000), source, kind,
                                  message.get_seq(), heartbeat))
    if not packets:
        raise ValueError(f"No decoded records: {path}")
    return packets, bad_data


def ordered_gaps(packets):
    return [(before, after) for before, after in pairwise(packets)
            if after.timestamp - before.timestamp > LIMIT_US]


def sequence_steps(packets):
    deltas = Counter((after.seq - before.seq) % 256 for before, after in pairwise(packets))
    return (deltas[1], deltas[0], sum(count for delta, count in deltas.items() if 2 <= delta <= 127),
            sum(count for delta, count in deltas.items() if delta >= 128))


def inside(packets, before, after):
    return [packet for packet in packets if before.timestamp < packet.timestamp < after.timestamp]


def match_gaps(first, second):
    candidates = []
    for left, (left_start, left_end) in enumerate(first):
        for right, (right_start, right_end) in enumerate(second):
            overlap = min(left_end.timestamp, right_end.timestamp) - max(left_start.timestamp, right_start.timestamp)
            if overlap > 0:
                candidates.append((overlap, left, right))
    used_left, used_right = set(), set()
    matches = []
    for overlap, left, right in sorted(candidates, reverse=True):
        if left not in used_left and right not in used_right:
            matches.append((left, right, overlap))
            used_left.add(left)
            used_right.add(right)
    return sorted(matches)


def clock(timestamp):
    return datetime.fromtimestamp(timestamp / 1_000_000, JST).isoformat(timespec="milliseconds")[11:23]


def span(before, after):
    return f"{clock(before.timestamp)} to {clock(after.timestamp)}"


def counts_text(counts):
    return ", ".join(f"{kind} x{count}" for kind, count in sorted(counts.items())) or "-"


def print_log(packets, bad_data, label):
    boat = [packet for packet in packets if packet.source == BOAT]
    gaps = ordered_gaps([packet for packet in boat if packet.heartbeat is not None])
    stamps = Counter(packet.timestamp for packet in packets)
    adjacent = [(before, after, after.timestamp - before.timestamp) for before, after in pairwise(packets)]
    print(f"\n## {label}\n")
    print(f"- {span(packets[0], packets[-1])} JST; {len(packets)} records; BAD_DATA={bad_data}; "
          f"backward steps={sum(delta < 0 for _, _, delta in adjacent)}; "
          f"equal adjacent={sum(delta == 0 for _, _, delta in adjacent)}; "
          f"repeated timestamp records={len(packets) - len(stamps)}.")
    print(f"- Sources: {dict(Counter(packet.source for packet in packets))}.")
    for name, stream in (("all records", packets), ("excluding 255/190", [p for p in packets if p.source != GCS]),
                         ("1/1 only", boat)):
        intervals = [(after.timestamp - before.timestamp, before, after) for before, after in pairwise(stream)]
        if intervals:
            duration, before, after = max(intervals, key=lambda entry: entry[0])
            print(f"- {name}: max {duration / 1e6:.6f} s at {span(before, after)} "
                  f"({before.kind} {before.source} -> {after.kind} {after.source}); "
                  f">3 s: {sum(delta > LIMIT_US for delta, _, _ in intervals)}.")
    print(f"- 1/1 sequence steps (+1, same, +2..127, +128..255 modulo 256): {sequence_steps(boat)}.")
    print("\n| # | HEARTBEAT interval (JST) | s | GPS / Position / EKF (1/1) | Longest 1/1 silence (JST / s / seq) | Other 1/1 | Other sources | 1/1 sequence steps (*) |")
    print("|---:|---|---:|---|---|---|---|---|")
    for index, (before, after) in enumerate(gaps, 1):
        interim = inside(packets, before, after)
        local = [packet for packet in interim if packet.source == BOAT]
        by_kind = Counter(packet.kind for packet in local)
        cores = " / ".join(str(by_kind.pop(kind, 0)) for kind in CORE)
        others = Counter(f"{packet.source[0]}/{packet.source[1]} {packet.kind}"
                         for packet in interim if packet.source != BOAT)
        duration = (after.timestamp - before.timestamp) / 1e6
        bounded = [before, *local, after]
        summary = sequence_steps(bounded)
        silent_start, silent_end = max(pairwise(bounded), key=lambda pair: pair[1].timestamp - pair[0].timestamp)
        silence = (silent_end.timestamp - silent_start.timestamp) / 1e6
        print(f"| {index} | {span(before, after)} | {duration:.3f} | {cores} | "
              f"{span(silent_start, silent_end)} / {silence:.3f} / {silent_start.seq}->{silent_end.seq} | {counts_text(by_kind)} | "
              f"{counts_text(others)} | {before.seq}->{after.seq}: {summary} |")
    print("\n(*) Steps compare adjacent 1/1 records, including the two bounding HEARTBEAT packets. "
          "Buckets are descriptive, not loss counts. Strictly interior packets only; bounds are excluded.")
    return gaps


def main():
    parser = argparse.ArgumentParser(description="Compare two local Mission Planner tlogs offline")
    parser.add_argument("tlogs", metavar="tlog", type=Path, nargs=2)
    args = parser.parse_args()
    try:
        logs = [read_tlog(path) for path in args.tlogs]
        print("# BlueBoat MAVLink gap analysis\n\nTimes are JST (UTC+09:00). Files are analyzed separately, not pooled.")
        gaps = [print_log(*data, path.name) for data, path in zip(logs, args.tlogs)]
        matches = match_gaps(*gaps)
        print(f"\n## Cross-log match: {len(matches)} pairs (by duration of overlap)\n")
        print("| A # | B # | Overlap s | B-A start ms | B-A end ms | Identical raw HEARTBEAT bytes: start / end |")
        print("|---:|---:|---:|---:|---:|---|")
        for left, right, overlap in matches:
            start_a, end_a = gaps[0][left]
            start_b, end_b = gaps[1][right]
            print(f"| {left+1} | {right+1} | {overlap / 1e6:.3f} | "
                  f"{(start_b.timestamp-start_a.timestamp) / 1000:+.3f} | "
                  f"{(end_b.timestamp-end_a.timestamp) / 1000:+.3f} | "
                  f"{start_a.heartbeat == start_b.heartbeat} / {end_a.heartbeat == end_b.heartbeat} |")
    except (ValueError, OSError) as exc:
        parser.exit(1, f"Analysis failed: {exc}\n")


if __name__ == "__main__":
    main()
