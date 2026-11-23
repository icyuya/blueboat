
import argparse
import time
from pymavlink import mavutil

CONNECTION = "udpin:0.0.0.0:14551"

# 初回試験用の低い操作入力
INPUT_LEVEL = 120

# 1回の回転指令は最大0.4秒
PULSE_SECONDS = 0.4
NEUTRAL_SECONDS = 1.0
HEARTBEAT_TIMEOUT = 2.5

ACTIONS = {
    "neutral": (0, 0),
    "forward": (INPUT_LEVEL, 0),
    "left": (0, -INPUT_LEVEL),
    "right": (0, INPUT_LEVEL),
}


def update_status(master, system_id, status):
    # 受信キューから最新のHeartbeatを確認する
    for _ in range(200):
        msg = master.recv_match(blocking=False)
        if msg is None:
            break

        if msg.get_type() != "HEARTBEAT":
            continue

        if msg.get_srcSystem() != system_id:
            continue

        if msg.autopilot != (
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
        ):
            continue

        status["mode"] = mavutil.mode_string_v10(msg)
        status["armed"] = bool(
            msg.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        status["last_heartbeat"] = time.monotonic()


def permitted(status, require_arm):
    if (
        time.monotonic() - status["last_heartbeat"]
        > HEARTBEAT_TIMEOUT
    ):
        return False, "Heartbeat timeout"

    if status["mode"] != "MANUAL":
        return False, "Mode is not MANUAL"

    if require_arm and not status["armed"]:
        return False, "Vehicle is DISARMED"

    return True, "OK"


def send_control(master, system_id, throttle, steering):
    # MANUAL_CONTROL: x, y, z, r, buttons
    master.mav.manual_control_send(
        system_id,
        0,
        0,
        throttle,
        steering,
        0
    )


def transmit(master, system_id, status,
             throttle, steering, duration, require_arm):

    end = time.monotonic() + duration

    while time.monotonic() < end:
        update_status(master, system_id, status)

        ok, reason = permitted(status, require_arm)
        if not ok:
            print("ABORT:", reason)
            return False

        send_control(
            master, system_id, throttle, steering
        )

        time.sleep(0.05)

    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "action", choices=ACTIONS.keys()
    )
    parser.add_argument(
        "--dry-run", action="store_true"
    )
    args = parser.parse_args()

    throttle, steering = ACTIONS[args.action]

    print("BlueBoat Thruster Test")
    print("Action:", args.action)
    print("Throttle:", throttle)
    print("Steering:", steering)

    if args.dry_run:
        print("DRY RUN: No MAVLink messages sent")
        return

    master = mavutil.mavlink_connection(
        CONNECTION,
        source_system=255,
        source_component=190
    )

    system_id = None
    status = {
        "mode": None,
        "armed": False,
        "last_heartbeat": -1e10
    }

    print("Waiting for ArduPilot Heartbeat...")

    try:
        deadline = time.monotonic() + 10

        while time.monotonic() < deadline:
            msg = master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1
            )

            if msg is None:
                continue

            if msg.autopilot != (
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            ):
                continue

            system_id = msg.get_srcSystem()
            status["mode"] = mavutil.mode_string_v10(msg)
            status["armed"] = bool(
                msg.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            status["last_heartbeat"] = time.monotonic()
            break

        if system_id is None:
            print("ERROR: No ArduPilot Heartbeat")
            return

        print("System ID:", system_id)
        print("Mode:", status["mode"])
        print("Armed:", status["armed"])

        require_arm = args.action != "neutral"
        ok, reason = permitted(status, require_arm)

        if not ok:
            print("BLOCKED:", reason)
            return

        if require_arm:
            confirmation = input(
                "プロペラ周囲の安全を確認しましたか？"
                " 実行する場合は START と入力: "
            )
            if confirmation != "START":
                print("Cancelled")
                return

        # まず中立入力を送る
        print("Sending initial neutral...")
        if not transmit(
            master, system_id, status,
            0, 0, NEUTRAL_SECONDS, require_arm
        ):
            return

        if args.action != "neutral":
            print("Sending short control pulse...")
            if not transmit(
                master, system_id, status,
                throttle, steering,
                PULSE_SECONDS, True
            ):
                return

        print("Returning to neutral...")

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        # MANUAL・通信維持の場合のみ中立を繰り返し送信
        if system_id is not None:
            update_status(master, system_id, status)
            ok, _ = permitted(status, False)

            if ok:
                try:
                    transmit(
                        master, system_id, status,
                        0, 0, NEUTRAL_SECONDS, False
                    )
                except Exception as exc:
                    print("Neutral send failed:", exc)
            else:
                print(
                    "Neutral not confirmed. "
                    "Check QGC and DISARM."
                )

        master.close()
        print("Test finished.")
        print("Verify motors stopped in QGC.")


if __name__ == "__main__":
    main()
