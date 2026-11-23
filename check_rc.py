
from pymavlink import mavutil
import time

CONNECTION = "udpin:0.0.0.0:14550"
DURATION = 60

master = mavutil.mavlink_connection(CONNECTION)

print("BlueBoat RC input monitor")
print("READ ONLY - ARM and control commands are not sent")
print("Waiting for MAVLink messages...\n")

start = time.monotonic()
last_print = 0
target_system = None

heartbeat_count = 0
rc_count = 0

try:
    while time.monotonic() - start < DURATION:

        msg = master.recv_match(
            type=["HEARTBEAT", "RC_CHANNELS"],
            blocking=True,
            timeout=1
        )

        if msg is None:
            continue

        kind = msg.get_type()

        if kind == "HEARTBEAT":

            if msg.autopilot != (
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            ):
                continue

            target_system = msg.get_srcSystem()
            heartbeat_count += 1

            armed = bool(
                msg.base_mode &
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

            if armed:
                print("WARNING: ARMED detected. Stop the test.")
                break

            if heartbeat_count == 1:
                print("System ID:", target_system)
                print("Mode:", mavutil.mode_string_v10(msg))
                print("Armed:", armed)
                print()

        elif kind == "RC_CHANNELS":

            if target_system is None:
                continue

            if msg.get_srcSystem() != target_system:
                continue

            rc_count += 1

            now = time.monotonic()

            if now - last_print < 1:
                continue

            last_print = now

            values = [
                getattr(msg, f"chan{i}_raw")
                for i in range(1, 9)
            ]

            print(
                f"CH1={values[0]} "
                f"CH2={values[1]} "
                f"CH3={values[2]} "
                f"CH4={values[3]} "
                f"CH5={values[4]} "
                f"CH6={values[5]} "
                f"CH7={values[6]} "
                f"CH8={values[7]}"
            )

finally:
    master.close()

print("\n--- Result ---")
print("HEARTBEAT:", heartbeat_count)
print("RC_CHANNELS:", rc_count)

if rc_count == 0:
    print("RC_CHANNELS was not received.")
    print("Check the MAVLink stream and RC input source.")
else:
    print("RC_CHANNELS messages were received.")
    print("Verify that values change with transmitter input.")
