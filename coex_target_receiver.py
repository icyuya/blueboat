import argparse
import re
import socket
import time
from ipaddress import IPv4Address

from blueboat_navigator import PORT, TARGET_RECV_BUFFER_BYTES, parse_target_packet


COEX_TIMEOUT_S = 3.0
MAX_SEQUENCE = 2**32 - 1
SESSION_ID = re.compile(
    r"[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}"
)


def _valid_session_id(value):
    return isinstance(value, str) and SESSION_ID.fullmatch(value) is not None


class CoexTargetGate:
    def __init__(self, expected_source, session_id):
        source_ip, source_port = expected_source
        if type(source_port) is not int or not 1 <= source_port <= 65535:
            raise ValueError("Source port must be in 1..65535")
        if not _valid_session_id(session_id):
            raise ValueError("Invalid session_id (expected lowercase UUIDv4)")

        self._expected_source = (str(IPv4Address(source_ip)), source_port)
        self._session_id = session_id
        self._last_seq = None
        self._last_received_at = None
        self._latch_reason = None

    @property
    def latch_reason(self):
        return self._latch_reason

    def check_timeout(self, monotonic_now):
        if (
            self._latch_reason is None
            and self._last_received_at is not None
            and monotonic_now - self._last_received_at >= COEX_TIMEOUT_S
        ):
            self._latch_reason = "COEX target timeout"
        return self._latch_reason

    def accept(self, raw, source, *, wall_now, monotonic_now):
        reason = self.check_timeout(monotonic_now)
        if reason is not None:
            raise ValueError(f"COEX LATCHED: {reason}")
        if source != self._expected_source:
            raise ValueError("Unexpected COEX source endpoint")

        target = parse_target_packet(raw, now=wall_now)
        session_id = target.get("session_id")
        seq = target.get("seq")
        if not _valid_session_id(session_id):
            raise ValueError("Invalid session_id (expected lowercase UUIDv4)")
        if type(seq) is not int or not 0 <= seq <= MAX_SEQUENCE:
            raise ValueError("Invalid seq (expected uint32)")

        if session_id != self._session_id:
            self._latch_reason = "Unexpected COEX session"
            raise ValueError(f"COEX LATCHED: {self._latch_reason}")
        if self._last_seq is not None:
            if seq == self._last_seq:
                raise ValueError("Duplicate COEX sequence")
            if seq < self._last_seq:
                raise ValueError("Out-of-order COEX sequence")

        self._last_seq = seq
        self._last_received_at = monotonic_now
        return target


def monitor_targets(receiver, gate):
    while True:
        try:
            raw, source = receiver.recvfrom(TARGET_RECV_BUFFER_BYTES)
        except socket.timeout:
            reason = gate.check_timeout(time.monotonic())
            if reason is not None:
                print("COEX LATCHED:", reason)
                return
            continue

        monotonic_now = time.monotonic()
        wall_now = time.time()
        try:
            target = gate.accept(
                raw, source, wall_now=wall_now, monotonic_now=monotonic_now,
            )
        except ValueError as exc:
            print("TARGET REJECTED:", exc)
            if gate.latch_reason is not None:
                return
            continue

        apparent_age = wall_now - target["sent_at"]
        print(f"TARGET ACCEPTED: seq={target['seq']} apparent_age={apparent_age:+.3f}s")


def main():
    parser = argparse.ArgumentParser(description="COEX diagnostics only; no vehicle connection")
    parser.add_argument("--listen-ip", type=IPv4Address, default="127.0.0.1")
    parser.add_argument("--source-ip", type=IPv4Address, required=True)
    parser.add_argument("--source-port", type=int, required=True)
    parser.add_argument("--session-id", required=True)
    args = parser.parse_args()
    try:
        gate = CoexTargetGate((args.source_ip, args.source_port), args.session_id)
    except ValueError as exc:
        parser.error(str(exc))

    print("COEX receive only - no vehicle connection or navigation commands")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind((str(args.listen_ip), PORT))
            receiver.settimeout(0.1)
            print(f"Listening on {args.listen_ip}:{PORT} for {args.source_ip}:{args.source_port}")
            monitor_targets(receiver, gate)
    except KeyboardInterrupt:
        print("COEX receiver stopped")


if __name__ == "__main__":
    main()
