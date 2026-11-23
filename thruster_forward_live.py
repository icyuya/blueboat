
import time
from pymavlink import mavutil

CONNECTION = "udpin:0.0.0.0:14551"
MAV = mavutil.mavlink

FORWARD = 20
PULSE_TIME = 0.2
NEUTRAL_PWM = 1510
PWM_TOLERANCE = 30

master = None
sysid = None

state = {
    "mode": None,
    "armed": False,
    "heartbeat": 0,
    "servo_time": 0,
    "servo1": None,
    "servo3": None,
}


def receive():
    for _ in range(200):
        msg = master.recv_match(blocking=False)
        if msg is None:
            break

        if msg.get_srcSystem() != sysid:
            continue

        kind = msg.get_type()

        if kind == "HEARTBEAT":
            if msg.autopilot != MAV.MAV_AUTOPILOT_ARDUPILOTMEGA:
                continue

            state["mode"] = mavutil.mode_string_v10(msg)
            state["armed"] = bool(
                msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED
            )
            state["heartbeat"] = time.monotonic()

        elif kind == "SERVO_OUTPUT_RAW":
            if msg.port != 0:
                continue

            state["servo1"] = msg.servo1_raw
            state["servo3"] = msg.servo3_raw
            state["servo_time"] = time.monotonic()


def check_safe():
    receive()

    if time.monotonic() - state["heartbeat"] > 2.5:
        return False, "Heartbeat timeout"

    if state["mode"] != "MANUAL":
        return False, "Not MANUAL"

    if not state["armed"]:
        return False, "DISARM detected"

    return True, "OK"


def send_control(z):
    master.mav.manual_control_send(
        sysid,
        0,  # x
        0,  # y: steering
        z,  # z: throttle
        0,  # r
        0,  # buttons
    )


def send_for(z, duration):
    end = time.monotonic() + duration
    count = 0

    while time.monotonic() < end:
        ok, reason = check_safe()
        if not ok:
            print("ABORT:", reason)
            return False

        send_control(z)
        count += 1
        time.sleep(0.05)

    receive()
    print("Sent messages:", count)
    return True


def main():
    global master, sysid

    print("BlueBoat Forward Live Test")
    print("Forward z=20 / Pulse=0.2s")

    master = mavutil.mavlink_connection(
        CONNECTION,
        source_system=255,
        source_component=190,
    )

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
            if hb.autopilot != MAV.MAV_AUTOPILOT_ARDUPILOTMEGA:
                continue

            sysid = hb.get_srcSystem()
            state["mode"] = mavutil.mode_string_v10(hb)
            state["armed"] = bool(
                hb.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED
            )
            state["heartbeat"] = time.monotonic()
            break

        if sysid is None:
            print("ERROR: No ArduRover heartbeat")
            return

        print("Mode:", state["mode"])
        print("Armed:", state["armed"])

        ok, reason = check_safe()
        if not ok:
            print("BLOCKED:", reason)
            return

        print("Sending initial neutral...")
        if not send_for(0, 0.6):
            return

        # 中立付近の実際の出力PWMを確認
        deadline = time.monotonic() + 2.0

        while time.monotonic() < deadline:
            receive()
            if (
                time.monotonic() - state["servo_time"]
                < 1.0
            ):
                break
            time.sleep(0.05)

        if (
            time.monotonic() - state["servo_time"]
            > 1.0
        ):
            print("BLOCKED: No fresh SERVO_OUTPUT_RAW")
            return

        s1 = state["servo1"]
        s3 = state["servo3"]

        print(f"Neutral PWM: Right={s1} Left={s3}")

        if (
            abs(s1 - NEUTRAL_PWM) > PWM_TOLERANCE
            or abs(s3 - NEUTRAL_PWM) > PWM_TOLERANCE
        ):
            print("BLOCKED: Output is not neutral")
            return

        answer = input(
            "前進パルスを送信します。"
            "実行する場合 START と入力: "
        )

        if answer.strip().upper() != "START":
            print("Cancelled")
            return

        ok, reason = check_safe()
        if not ok:
            print("BLOCKED:", reason)
            return

        print("FORWARD z=20")
        if not send_for(FORWARD, PULSE_TIME):
            return

        print(
            "Last observed PWM:",
            state["servo1"],
            state["servo3"],
        )

    except KeyboardInterrupt:
        print("Interrupted")

    finally:
        if sysid is not None:
            print("Returning to NEUTRAL z=0")
            try:
                for _ in range(20):
                    send_control(0)
                    time.sleep(0.05)
                print("Neutral commands sent")
            except Exception as exc:
                print("WARNING:", exc)

        master.close()
        print("Finished - DISARM in QGC")


if __name__ == "__main__":
    main()
