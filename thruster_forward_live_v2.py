
import argparse
import time
from pymavlink import mavutil

LINK = "udpin:0.0.0.0:14551"
MAV = mavutil.mavlink

FORWARD_Z = 180
PULSE_SECONDS = 0.6
NEUTRAL_Z = 0
NEUTRAL_PWM = 1510
PWM_TOLERANCE = 20
HEARTBEAT_TIMEOUT = 2.5

parser = argparse.ArgumentParser()
parser.add_argument("--execute", action="store_true")
args = parser.parse_args()

print("BlueBoat Forward Live Test v2")
print(f"Forward z={FORWARD_Z}, duration={PULSE_SECONDS}s")

if not args.execute:
    print("DRY RUN: No commands sent")
    raise SystemExit(0)

master = mavutil.mavlink_connection(
    LINK,
    source_system=255,
    source_component=190
)

sysid = None
state = {
    "mode": None,
    "armed": False,
    "last_hb": 0.0,
    "last_servo": 0.0,
    "servo1": None,
    "servo3": None,
    "rc3": None,
    "pwm_samples": [],
}

phase = "WAIT"


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
                msg.base_mode
                & MAV.MAV_MODE_FLAG_SAFETY_ARMED
            )
            state["last_hb"] = time.monotonic()

        elif kind == "SERVO_OUTPUT_RAW" and msg.port == 0:
            state["servo1"] = msg.servo1_raw
            state["servo3"] = msg.servo3_raw
            state["last_servo"] = time.monotonic()

            if phase == "FORWARD":
                state["pwm_samples"].append(
                    (msg.servo1_raw, msg.servo3_raw)
                )
                print(
                    f"PWM: Right={msg.servo1_raw} "
                    f"Left={msg.servo3_raw}"
                )

        elif kind == "RC_CHANNELS":
            state["rc3"] = msg.chan3_raw


def allowed():
    receive()

    if time.monotonic() - state["last_hb"] > HEARTBEAT_TIMEOUT:
        return False, "Heartbeat timeout"

    if state["mode"] != "MANUAL":
        return False, "Mode is not MANUAL"

    if not state["armed"]:
        return False, "Vehicle DISARMED"

    return True, "OK"


def send(z):
    master.mav.manual_control_send(
        sysid,
        0,  # x
        0,  # y: steering
        z,  # z: throttle
        0,  # r
        0,  # buttons
    )


def send_for(z, seconds):
    end = time.monotonic() + seconds
    count = 0

    while time.monotonic() < end:
        ok, reason = allowed()
        if not ok:
            print("ABORT:", reason)
            return False

        send(z)
        count += 1
        time.sleep(0.05)

    receive()
    print("Messages sent:", count)
    return True


try:
    print("Waiting for ArduRover heartbeat...")

    deadline = time.monotonic() + 10

    while time.monotonic() < deadline:
        hb = master.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1
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
        state["last_hb"] = time.monotonic()
        break

    if sysid is None:
        print("ERROR: No ArduRover heartbeat")
        raise SystemExit(1)

    print("Mode:", state["mode"])
    print("Armed:", state["armed"])

    ok, reason = allowed()
    if not ok:
        print("BLOCKED:", reason)
        raise SystemExit(1)

    print("Initial neutral z=0")
    if not send_for(0, 0.8):
        raise SystemExit(1)

    receive()

    s1 = state["servo1"]
    s3 = state["servo3"]

    print("Current PWM:", s1, s3)

    fresh = (
        time.monotonic() - state["last_servo"] < 1.2
    )

    if not fresh or s1 is None or s3 is None:
        print("BLOCKED: No fresh servo data")
        raise SystemExit(1)

    if (
        abs(s1 - NEUTRAL_PWM) > PWM_TOLERANCE
        or abs(s3 - NEUTRAL_PWM) > PWM_TOLERANCE
    ):
        print("BLOCKED: Outputs are not neutral")
        raise SystemExit(1)

    answer = input(
        "短い前進指令を送信します。START と入力: "
    )

    if answer.strip().upper() != "START":
        print("Cancelled")
        raise SystemExit(0)

    ok, reason = allowed()
    if not ok:
        print("BLOCKED:", reason)
        raise SystemExit(1)

    print("Refreshing neutral")
    if not send_for(0, 0.2):
        raise SystemExit(1)

    phase = "FORWARD"
    print(f"FORWARD z={FORWARD_Z}")

    success = send_for(FORWARD_Z, PULSE_SECONDS)
    phase = "NEUTRAL"

    if not success:
        print("Forward test aborted")

except KeyboardInterrupt:
    print("Interrupted")

finally:
    if sysid is not None:
        receive()

        link_ok = (
            time.monotonic() - state["last_hb"]
            <= HEARTBEAT_TIMEOUT
        )

        if state["mode"] == "MANUAL" and link_ok:
            print("Returning to NEUTRAL z=0")
            try:
                for _ in range(20):
                    receive()
                    if state["mode"] != "MANUAL":
                        break
                    send(NEUTRAL_Z)
                    time.sleep(0.05)
            except Exception as exc:
                print("Neutral send error:", exc)
        else:
            print("Neutral not confirmed; DISARM using QGC")

    samples = state["pwm_samples"]

    print("\n--- RESULT ---")
    print("Mode:", state["mode"])
    print("Armed:", state["armed"])
    print("Last RC3:", state["rc3"])
    print("Last PWM:", state["servo1"], state["servo3"])

    if samples:
        print("PWM samples during forward:", len(samples))
        print(
            "Right PWM range:",
            min(a for a, b in samples),
            max(a for a, b in samples)
        )
        print(
            "Left PWM range:",
            min(b for a, b in samples),
            max(b for a, b in samples)
        )
    else:
        print("No PWM sample during short forward pulse")

    master.close()
    print("Finished - DISARM using QGC")
