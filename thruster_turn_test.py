
import argparse
import time
from pymavlink import mavutil

LINK = "udpin:0.0.0.0:14551"
MAV = mavutil.mavlink

STEERING = {
    "left": -180,
    "right": 180,
}

PULSE_SECONDS = 0.6
NEUTRAL_PWM = 1510
PWM_TOLERANCE = 20
HB_TIMEOUT = 2.5


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "direction", choices=["left", "right"]
    )
    parser.add_argument(
        "--execute", action="store_true"
    )
    args = parser.parse_args()

    steering = STEERING[args.direction]

    print("BlueBoat Steering Test")
    print(f"Direction: {args.direction}")
    print(f"Steering y={steering}, throttle z=0")

    if not args.execute:
        print("DRY RUN - no commands sent")
        return

    master = mavutil.mavlink_connection(
        LINK,
        source_system=255,
        source_component=190,
    )

    sysid = None
    phase = "WAIT"

    state = {
        "mode": None,
        "armed": False,
        "heartbeat": 0.0,
        "servo_time": 0.0,
        "right": None,
        "left": None,
        "samples": [],
    }

    def receive():
        nonlocal phase

        for _ in range(200):
            msg = master.recv_match(blocking=False)

            if msg is None:
                break

            if msg.get_srcSystem() != sysid:
                continue

            kind = msg.get_type()

            if kind == "HEARTBEAT":
                if msg.autopilot != (
                    MAV.MAV_AUTOPILOT_ARDUPILOTMEGA
                ):
                    continue

                state["mode"] = mavutil.mode_string_v10(msg)
                state["armed"] = bool(
                    msg.base_mode
                    & MAV.MAV_MODE_FLAG_SAFETY_ARMED
                )
                state["heartbeat"] = time.monotonic()

            elif kind == "SERVO_OUTPUT_RAW":
                if msg.port != 0:
                    continue

                state["right"] = msg.servo1_raw
                state["left"] = msg.servo3_raw
                state["servo_time"] = time.monotonic()

                if phase == "STEERING":
                    state["samples"].append(
                        (msg.servo1_raw, msg.servo3_raw)
                    )

                    print(
                        f"PWM: Right={msg.servo1_raw} "
                        f"Left={msg.servo3_raw}"
                    )

    def allowed():
        receive()

        if (
            time.monotonic() - state["heartbeat"]
            > HB_TIMEOUT
        ):
            return False, "Heartbeat timeout"

        if state["mode"] != "MANUAL":
            return False, "Not MANUAL"

        if not state["armed"]:
            return False, "DISARMED"

        return True, "OK"

    def send(y):
        master.mav.manual_control_send(
            sysid,
            0,  # x
            y,  # y: steering
            0,  # z: neutral throttle
            0,  # r
            0,  # buttons
        )

    def send_for(y, duration):
        end = time.monotonic() + duration
        count = 0

        while time.monotonic() < end:
            ok, reason = allowed()

            if not ok:
                print("ABORT:", reason)
                return False

            send(y)
            count += 1
            time.sleep(0.05)

        receive()
        print("Messages sent:", count)
        return True

    try:
        print("Waiting for ArduRover...")

        deadline = time.monotonic() + 10

        while time.monotonic() < deadline:
            hb = master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1,
            )

            if hb is None:
                continue

            if hb.autopilot != (
                MAV.MAV_AUTOPILOT_ARDUPILOTMEGA
            ):
                continue

            sysid = hb.get_srcSystem()
            state["mode"] = mavutil.mode_string_v10(hb)
            state["armed"] = bool(
                hb.base_mode
                & MAV.MAV_MODE_FLAG_SAFETY_ARMED
            )
            state["heartbeat"] = time.monotonic()
            break

        if sysid is None:
            print("ERROR: No heartbeat")
            return

        print("Mode:", state["mode"])
        print("Armed:", state["armed"])

        ok, reason = allowed()
        if not ok:
            print("BLOCKED:", reason)
            return

        print("Sending initial neutral")

        if not send_for(0, 0.8):
            return

        receive()

        if (
            time.monotonic() - state["servo_time"]
            > 1.0
            or state["right"] is None
            or state["left"] is None
        ):
            print("BLOCKED: No fresh servo data")
            return

        print(
            "Neutral PWM:",
            state["right"],
            state["left"],
        )

        if (
            abs(state["right"] - NEUTRAL_PWM)
            > PWM_TOLERANCE
            or abs(state["left"] - NEUTRAL_PWM)
            > PWM_TOLERANCE
        ):
            print("BLOCKED: PWM not neutral")
            return

        answer = input(
            "旋回指令を送ります。START と入力: "
        )

        if answer.strip().upper() != "START":
            print("Cancelled")
            return

        if not send_for(0, 0.2):
            return

        phase = "STEERING"

        print(f"STEERING y={steering}")

        send_for(steering, PULSE_SECONDS)

        phase = "NEUTRAL"

    except KeyboardInterrupt:
        print("Interrupted")

    finally:
        if sysid is not None:
            receive()

            link_ok = (
                time.monotonic() - state["heartbeat"]
                <= HB_TIMEOUT
            )

            if state["mode"] == "MANUAL" and link_ok:
                print("Returning to neutral")

                try:
                    end = time.monotonic() + 1.0

                    while time.monotonic() < end:
                        receive()

                        if (
                            state["mode"] != "MANUAL"
                            or not state["armed"]
                        ):
                            break

                        send(0)
                        time.sleep(0.05)

                except Exception as exc:
                    print("Neutral error:", exc)

            else:
                print("Neutral not confirmed")

        print("\n--- RESULT ---")
        print("Mode:", state["mode"])
        print("Armed:", state["armed"])
        print(
            "Last PWM:",
            state["right"],
            state["left"],
        )

        samples = state["samples"]

        if samples:
            print("PWM samples:", len(samples))

            print(
                "Right range:",
                min(s[0] for s in samples),
                max(s[0] for s in samples),
            )

            print(
                "Left range:",
                min(s[1] for s in samples),
                max(s[1] for s in samples),
            )
        else:
            print("No PWM samples during steering")

        master.close()
        print("Finished - DISARM using QGC")


if __name__ == "__main__":
    main()
