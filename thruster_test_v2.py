
import argparse
import time
from pymavlink import mavutil

CONNECTION = "udpin:0.0.0.0:14551"
MAV = mavutil.mavlink

# Roverでは z=500 が中立
ACTIONS = {
    "neutral": (500, 0),
    "forward": (520, 0),
    "left": (500, -20),
    "right": (500, 20),
}

PULSE_SECONDS = 0.3
NEUTRAL_SECONDS = 1.0
HEARTBEAT_TIMEOUT = 2.5


def get_status(master, status):
    for _ in range(200):
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        if msg.get_type() != "HEARTBEAT":
            continue
        if msg.autopilot != MAV.MAV_AUTOPILOT_ARDUPILOTMEGA:
            continue
        if msg.get_srcSystem() != status["sysid"]:
            continue

        status["mode"] = mavutil.mode_string_v10(msg)
        status["armed"] = bool(
            msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED
        )
        status["last_hb"] = time.monotonic()


def permitted(status, require_arm):
    if time.monotonic() - status["last_hb"] > HEARTBEAT_TIMEOUT:
        return False
    if status["mode"] != "MANUAL":
        return False
    if require_arm and not status["armed"]:
        return False
    return True


def send_control(master, sysid, throttle, steering):
    # x=0, y=steering, z=throttle, r=0
    master.mav.manual_control_send(
        sysid, 0, steering, throttle, 0, 0
    )


def transmit(master, status, throttle, steering, seconds,
             require_arm):
    end = time.monotonic() + seconds

    while time.monotonic() < end:
        get_status(master, status)

        if not permitted(status, require_arm):
            print("ABORT: Mode, ARM or heartbeat changed")
            return False

        send_control(
            master, status["sysid"], throttle, steering
        )
        time.sleep(0.05)

    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=ACTIONS)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    throttle, steering = ACTIONS[args.action]
    print(f"Action={args.action} z={throttle} y={steering}")

    # 標準はDRY RUN
    if not args.execute:
        print("DRY RUN: No MAVLink messages sent")
        return

    print("LIVE: Real MAVLink control messages")
    master = mavutil.mavlink_connection(
        CONNECTION,
        source_system=255,
        source_component=190
    )

    status = {
        "sysid": None,
        "mode": None,
        "armed": False,
        "last_hb": -1e10,
    }

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
            if msg.autopilot != MAV.MAV_AUTOPILOT_ARDUPILOTMEGA:
                continue

            status["sysid"] = msg.get_srcSystem()
            status["mode"] = mavutil.mode_string_v10(msg)
            status["armed"] = bool(
                msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED
            )
            status["last_hb"] = time.monotonic()
            break

        if status["sysid"] is None:
            print("ERROR: No ArduPilot Heartbeat")
            return

        print("Mode:", status["mode"])
        print("Armed:", status["armed"])

        require_arm = args.action != "neutral"

        if not permitted(status, require_arm):
            print("BLOCKED: Safety conditions not satisfied")
            return

        answer = input("実機へ送信する場合 START と入力: ")
        if answer != "START":
            print("Cancelled")
            return

        if args.action == "neutral":
            transmit(
                master, status, 500, 0,
                NEUTRAL_SECONDS, False
            )
        else:
            print("Sending short pulse")
            transmit(
                master, status, throttle, steering,
                PULSE_SECONDS, True
            )

    except KeyboardInterrupt:
        print("Interrupted")

    finally:
        if status["sysid"] is not None:
            try:
                get_status(master, status)
                if permitted(status, False):
                    print("Sending neutral")
                    transmit(
                        master, status, 500, 0,
                        NEUTRAL_SECONDS, False
                    )
                else:
                    print("Neutral not confirmed - check QGC")
            except Exception as exc:
                print("Neutral send error:", exc)

        master.close()
        print("Finished")


if __name__ == "__main__":
    main()
