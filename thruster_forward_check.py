
import time
from pymavlink import mavutil

# ==========================================
# BlueBoat Forward Input Check
# ArduRover 4.7.0
# DISARM ONLY
# ==========================================

CONNECTION = "udpin:0.0.0.0:14551"
MAV = mavutil.mavlink

SOURCE_SYSTEM = 255

# Test settings
THROTTLE = 80
PULSE_SECONDS = 3.0

HEARTBEAT_TIMEOUT = 2.5
SEND_INTERVAL = 0.05
NEUTRAL_SECONDS = 1.0
OVERRIDE_TIMEOUT_WAIT = 3.5


def main():
    print("==================================")
    print(" BlueBoat Forward Input Test")
    print(" DISARM ONLY")
    print("==================================")

    print(f"Throttle: z={THROTTLE}")
    print(f"Duration: {PULSE_SECONDS} seconds")
    print()

    master = mavutil.mavlink_connection(
        CONNECTION,
        source_system=SOURCE_SYSTEM,
        source_component=190,
    )

    sysid = None

    state = {
        "mode": None,
        "armed": False,
        "heartbeat": 0.0,
        "rc_count": None,
        "rc3": None,
        "rc_samples": [],
    }

    # --------------------------------------
    # Receive MAVLink messages
    # --------------------------------------
    def receive():
        while True:
            msg = master.recv_match(blocking=False)

            if msg is None:
                break

            if msg.get_srcSystem() != sysid:
                continue

            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                if (
                    msg.autopilot
                    != MAV.MAV_AUTOPILOT_ARDUPILOTMEGA
                ):
                    continue

                state["mode"] = mavutil.mode_string_v10(msg)

                state["armed"] = bool(
                    msg.base_mode
                    & MAV.MAV_MODE_FLAG_SAFETY_ARMED
                )

                state["heartbeat"] = time.monotonic()

            elif msg_type == "RC_CHANNELS":
                state["rc_count"] = msg.chancount
                state["rc3"] = msg.chan3_raw

                state["rc_samples"].append(
                    (time.monotonic(), msg.chan3_raw)
                )

                print(
                    f"RC INPUT: "
                    f"count={msg.chancount} "
                    f"throttle={msg.chan3_raw}"
                )

    # --------------------------------------
    # Safety check
    # --------------------------------------
    def allowed():
        if state["mode"] != "MANUAL":
            return False, "Not in MANUAL mode"

        if state["armed"]:
            return False, "Vehicle is ARMED"

        if (
            time.monotonic() - state["heartbeat"]
            > HEARTBEAT_TIMEOUT
        ):
            return False, "Heartbeat timeout"

        return True, "OK"

    # --------------------------------------
    # Send MANUAL_CONTROL
    # --------------------------------------
    def send_control(z):
        master.mav.manual_control_send(
            sysid,
            0,  # x
            0,  # y: steering
            z,  # z: throttle
            0,  # r
            0,  # buttons
        )

    try:
        print("Waiting for ArduRover heartbeat...")

        deadline = time.monotonic() + 10.0

        while time.monotonic() < deadline:
            msg = master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1,
            )

            if msg is None:
                continue

            if (
                msg.autopilot
                != MAV.MAV_AUTOPILOT_ARDUPILOTMEGA
            ):
                continue

            sysid = msg.get_srcSystem()

            state["mode"] = mavutil.mode_string_v10(msg)

            state["armed"] = bool(
                msg.base_mode
                & MAV.MAV_MODE_FLAG_SAFETY_ARMED
            )

            state["heartbeat"] = time.monotonic()
            break

        if sysid is None:
            print("ERROR: No ArduRover heartbeat")
            return

        print("System ID:", sysid)
        print("Mode:", state["mode"])
        print("Armed:", state["armed"])

        receive()

        ok, reason = allowed()

        if not ok:
            print("BLOCKED:", reason)
            return

        # ----------------------------------
        # Forward input test
        # ----------------------------------
        print()
        print(
            f"Sending forward input "
            f"z={THROTTLE} "
            f"for {PULSE_SECONDS} seconds"
        )

        end = time.monotonic() + PULSE_SECONDS
        count = 0

        while time.monotonic() < end:
            receive()

            ok, reason = allowed()

            if not ok:
                print("ABORT:", reason)
                return

            send_control(THROTTLE)

            count += 1
            time.sleep(SEND_INTERVAL)

        print()
        print("Forward messages sent:", count)

    except KeyboardInterrupt:
        print()
        print("Interrupted by user")

    finally:
        # ----------------------------------
        # Return to neutral
        # ----------------------------------
        if sysid is not None:
            print()
            print("Returning to NEUTRAL z=0")

            try:
                receive()

                link_recent = (
                    time.monotonic() - state["heartbeat"]
                    <= HEARTBEAT_TIMEOUT
                )

                if state["mode"] == "MANUAL" and link_recent:
                    neutral_end = (
                        time.monotonic() + NEUTRAL_SECONDS
                    )

                    while time.monotonic() < neutral_end:
                        send_control(0)
                        time.sleep(SEND_INTERVAL)

                    print("Neutral commands sent")
                else:
                    print(
                        "WARNING: Neutral delivery "
                        "not confirmed"
                    )

            except Exception as exc:
                print("WARNING:", exc)

        # ----------------------------------
        # Wait for input timeout
        # ----------------------------------
        print()
        print("Waiting for input timeout...")

        time.sleep(OVERRIDE_TIMEOUT_WAIT)

        if sysid is not None:
            receive()

        # ----------------------------------
        # Results
        # ----------------------------------
        print()
        print("========== RESULTS ==========")

        print("Mode:", state["mode"])
        print("Armed:", state["armed"])

        print(
            "RC channel count:",
            state["rc_count"]
        )

        print(
            "RC channel 3:",
            state["rc3"]
        )

        samples = state["rc_samples"]

        if samples:
            values = [value for _, value in samples]

            print("RC samples:", len(values))
            print("RC minimum:", min(values))
            print("RC maximum:", max(values))
        else:
            print("No RC_CHANNELS samples received")

        print("=============================")

        master.close()

        print("Finished")
        print("Keep vehicle DISARMED")


if __name__ == "__main__":
    main()
