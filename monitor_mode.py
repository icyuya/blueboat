
from pymavlink import mavutil
from datetime import datetime
import time

# Python専用MAVLink Endpoint
CONNECTION = "udpin:0.0.0.0:14551"

master = mavutil.mavlink_connection(CONNECTION)

print("BlueBoat Mode Monitor")
print("READ ONLY - No ARM or movement commands")
print("Waiting for Heartbeat...")

last_mode = None
last_heartbeat = time.monotonic()
connected = False

try:
    while True:
        msg = master.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1
        )

        if msg is not None:
            if msg.autopilot != (
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            ):
                continue

            connected = True
            last_heartbeat = time.monotonic()

            mode = mavutil.mode_string_v10(msg)

            armed = bool(
                msg.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

            if mode != last_mode:
                print(
                    datetime.now().strftime("%H:%M:%S"),
                    "Mode:", mode,
                    "Armed:", armed
                )

                if last_mode is not None and mode in ("MANUAL", "HOLD"):
                    print("Manual/Hold mode change detected")

                last_mode = mode

            if armed:
                print("WARNING: ARMED detected")
                break

        if time.monotonic() - last_heartbeat > 5:
            if connected:
                print("WARNING: MAVLink Heartbeat timeout")
            else:
                print("ERROR: No Heartbeat received on UDP 14551")
            break

except KeyboardInterrupt:
    print("Monitoring stopped")

finally:
    master.close()
