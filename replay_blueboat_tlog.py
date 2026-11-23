import argparse
import csv
import logging
import math
import statistics
from collections import Counter
from contextlib import closing
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

from pymavlink import mavutil

from blueboat_navigator_live_v3 import BOAT_COMPID, BOAT_SYSID, MAV, DryRunNavigator


KINDS = ("HEARTBEAT", "GPS_RAW_INT", "GLOBAL_POSITION_INT", "EKF_STATUS_REPORT")
TIMED_KINDS = ("EKF_STATUS_REPORT", "HEARTBEAT")
MICROS_PER_SECOND = 1_000_000
POLL_US = 100_000
GAP_LIMIT_US = 3 * MICROS_PER_SECOND
UNUSED_COEX_SESSION = "a0000000-0000-4000-8000-000000000000"
CSV_FIELDS = (
    "event", "kind", "elapsed_s", "gap_start_s", "gap_s", "intervals",
    "median_s", "p95_s", "p99_s", "max_s", "over_3s", "detail",
)


def seconds(microseconds):
    return microseconds / MICROS_PER_SECOND


@dataclass
class ReplayResult:
    origin_us: int
    end_us: int = 0
    counts: Counter = field(default_factory=Counter)
    bad_packets: int = 0
    arrivals: dict = field(default_factory=lambda: {kind: [] for kind in TIMED_KINDS})
    events: list = field(default_factory=list)


def is_target(message, kind):
    if kind not in KINDS or (message.get_srcSystem(), message.get_srcComponent()) != (BOAT_SYSID, BOAT_COMPID):
        return False
    return kind != "HEARTBEAT" or (
        message.type == MAV.MAV_TYPE_SURFACE_BOAT
        and message.autopilot == MAV.MAV_AUTOPILOT_ARDUPILOTMEGA
    )


def replay(messages):
    navigator = DryRunNavigator(coex_source=("127.0.0.1", 16001), coex_session_id=UNUSED_COEX_SESSION)
    result = None
    next_poll_us = 0
    previous_reasons = None
    first_latch = None
    bad_packets = 0

    def snapshot(elapsed_us):
        nonlocal previous_reasons, first_latch
        reasons = navigator.poll(monotonic_now=seconds(elapsed_us))
        if navigator.latch_reason is not None and first_latch is None:
            first_latch = navigator.latch_reason
            result.events.append(("latch", elapsed_us, first_latch))
        if reasons != previous_reasons:
            result.events.append(("state", elapsed_us, "; ".join(reasons) or "Telemetry checks clear"))
            previous_reasons = reasons

    for message in messages:
        kind = message.get_type()
        if kind == "BAD_DATA":
            bad_packets += 1
            continue

        timestamp = getattr(message, "_timestamp", None)
        if not isinstance(timestamp, (float, int)) or not math.isfinite(timestamp) or timestamp <= 0:
            raise ValueError("Missing or invalid tlog receive timestamp")
        timestamp_us = round(timestamp * MICROS_PER_SECOND)
        if result is None:
            result = ReplayResult(timestamp_us)
        elapsed_us = timestamp_us - result.origin_us
        if elapsed_us < result.end_us:
            raise ValueError(f"Tlog receive timestamps move backwards at +{seconds(elapsed_us):.6f} s")
        result.end_us = elapsed_us

        while next_poll_us <= elapsed_us:
            snapshot(next_poll_us)
            next_poll_us += POLL_US

        if not is_target(message, kind):
            continue
        result.counts[kind] += 1
        if kind in result.arrivals:
            result.arrivals[kind].append(elapsed_us)
        snapshot(elapsed_us)
        navigator.observe_telemetry(message, received_at=seconds(elapsed_us))
        snapshot(elapsed_us)

    if result is None or not result.counts:
        raise ValueError("No matching timestamped telemetry for system 1, component 1")
    result.bad_packets = bad_packets
    snapshot(result.end_us)
    return result


def read_tlog(path):
    if path.suffix.lower() != ".tlog" or not path.is_file():
        raise ValueError("Input must be a local .tlog file")
    with closing(mavutil.mavlogfile(str(path), write=False, notimestamps=False)) as log:
        return replay(iter(log.recv_msg, None))


def percentile(sorted_values, percent):
    if not sorted_values:
        return None
    index = (len(sorted_values) - 1) * percent / 100
    lower = math.floor(index)
    upper = math.ceil(index)
    return sorted_values[lower] + (sorted_values[upper] - sorted_values[lower]) * (index - lower)


def report_rows(result):
    origin_utc = datetime.fromtimestamp(seconds(result.origin_us), timezone.utc).isoformat()
    yield {"event": "log", "elapsed_s": seconds(result.end_us), "detail": origin_utc}
    for kind in KINDS:
        yield {"event": "messages", "kind": kind, "detail": result.counts[kind]}
    yield {"event": "bad_data", "detail": result.bad_packets}
    for event, elapsed_us, detail in result.events:
        yield {"event": event, "elapsed_s": seconds(elapsed_us), "detail": detail}
    for kind, arrivals in result.arrivals.items():
        gaps = [(end - start, start, end) for start, end in zip(arrivals, arrivals[1:])]
        durations = sorted(seconds(gap) for gap, _, _ in gaps)
        over_limit = [(gap, start, end) for gap, start, end in gaps if gap > GAP_LIMIT_US]
        yield {
            "event": "interval_stats", "kind": kind, "intervals": len(durations),
            "median_s": statistics.median(durations) if durations else None,
            "p95_s": percentile(durations, 95), "p99_s": percentile(durations, 99),
            "max_s": durations[-1] if durations else None, "over_3s": len(over_limit),
        }
        for gap, start, end in over_limit:
            yield {
                "event": "gap_over_3s", "kind": kind, "elapsed_s": seconds(end),
                "gap_start_s": seconds(start), "gap_s": seconds(gap),
            }


def print_report(rows):
    def formatted(value):
        return "n/a" if value is None else f"{value:.3f} s"

    for row in rows:
        event = row["event"]
        if event == "log":
            print(f"Start UTC: {row['detail']} | duration: {formatted(row['elapsed_s'])}")
        elif event == "messages":
            print(f"{row['kind']}: {row['detail']} messages")
        elif event == "bad_data":
            print(f"Skipped BAD_DATA records: {row['detail']}")
        elif event in ("state", "latch"):
            print(f"+{row['elapsed_s']:.3f} s {event.upper()}: {row['detail']}")
        elif event == "interval_stats":
            metrics = ", ".join(
                f"{key}={formatted(row[key + '_s'])}" for key in ("median", "p95", "p99", "max")
            )
            print(f"{row['kind']}: {row['intervals']} intervals, {metrics}, >3 s: {row['over_3s']}")
        elif event == "gap_over_3s":
            print(f"  {row['kind']} gap: +{row['gap_start_s']:.3f} to +{row['elapsed_s']:.3f} s ({formatted(row['gap_s'])})")
    if not any(row["event"] == "latch" for row in rows):
        print("No safety latch during this recording")
    print("COEX targets: not replayed; waypoint generation not invoked")


def main():
    parser = argparse.ArgumentParser(description="Replay one local .tlog offline; no vehicle connection")
    parser.add_argument("tlog", type=Path)
    parser.add_argument("--csv", type=Path, help="write a new event/statistics CSV (no overwrite)")
    args = parser.parse_args()
    logging.basicConfig(level=logging.CRITICAL)
    try:
        rows = list(report_rows(read_tlog(args.tlog)))
        print(f"DRY RUN: {args.tlog}")
        print_report(rows)
        if args.csv:
            with args.csv.open("x", encoding="utf-8-sig", newline="") as output:
                writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
                writer.writeheader()
                writer.writerows(rows)
            print(f"CSV: {args.csv}")
    except (OSError, ValueError, OverflowError) as exc:
        parser.exit(1, f"Replay failed: {exc}\n")


if __name__ == "__main__":
    main()
