import time
from pymavlink import mavutil

MAVLINK = "udpin:0.0.0.0:14551"
HEARTBEAT_TIMEOUT = 3.0

def main():
    print("=== BlueBoat Safety Monitor ===")
    print("READ ONLY - No commands transmitted")

    master = mavutil.mavlink_connection(MAVLINK)

    last_heartbeat = None
    last_status = None
    mode = None
    armed = None
    safety_latched = False

    try:
        while True:
            msg = master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=0.5
            )

            now = time.monotonic()

            # Check for communication loss
            if last_heartbeat is not None:
                if now - last_heartbeat > HEARTBEAT_TIMEOUT:
                    safety_latched = True

            if msg is not None:
                # Accept only the BlueBoat autopilot
                if (
                    msg.get_srcSystem() == 1
                    and msg.get_srcComponent() == 1
                    and msg.type == mavutil.mavlink.MAV_TYPE_SURFACE_BOAT
                    and msg.autopilot ==
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                ):
                    last_heartbeat = now

                    mode = mavutil.mode_string_v10(msg)

                    armed = bool(
                        msg.base_mode &
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )

                    # Latch if the expected test state is lost
                    if mode not in ("MANUAL", "HOLD") or armed:
                        safety_latched = True

            if last_heartbeat is None:
                status = "WAITING FOR HEARTBEAT"
            elif safety_latched:
                status = "SAFETY LATCHED"
            else:
                status = "MONITOR OK"

            if status != last_status:
                print(f"\nStatus: {status}")
                print(f"Mode: {mode}")
                print(f"Armed: {armed}")
                last_status = status

    except KeyboardInterrupt:
        print("\nMonitor stopped")

    finally:
        master.close()


if __name__ == "__main__":
    main()