
import time
from pymavlink import mavutil

CONNECTION = "udpin:0.0.0.0:14551"
MAV = mavutil.mavlink

# 第1段階：中立のみ許可
THROTTLE = 0
STEERING = 0
SEND_DURATION = 1.0


def main():
    print("BlueBoat Thruster Control v3")
    print("NEUTRAL ONLY / DISARM REQUIRED")
    print("Connecting...")

    master = mavutil.mavlink_connection(
        CONNECTION,
        source_system=255,
        source_component=190
    )

    try:
        # ArduRoverのHeartbeatを取得
        heartbeat = None
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

            heartbeat = msg
            break

        if heartbeat is None:
            print("ERROR: No ArduPilot Heartbeat")
            return

        sysid = heartbeat.get_srcSystem()
        mode = mavutil.mode_string_v10(heartbeat)
        armed = bool(
            heartbeat.base_mode
            & MAV.MAV_MODE_FLAG_SAFETY_ARMED
        )

        print("System ID:", sysid)
        print("Mode:", mode)
        print("Armed:", armed)

        if armed:
            print("BLOCKED: DISARM required")
            return

        if mode != "MANUAL":
            print("BLOCKED: MANUAL mode required")
            return

        print("Sending NEUTRAL: y=0 z=0")

        end = time.monotonic() + SEND_DURATION
        count = 0

        while time.monotonic() < end:
            # 受信済みHeartbeatからARM・モード変更を監視
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break

                if msg.get_type() != "HEARTBEAT":
                    continue

                if msg.get_srcSystem() != sysid:
                    continue

                if msg.autopilot != MAV.MAV_AUTOPILOT_ARDUPILOTMEGA:
                    continue

                mode = mavutil.mode_string_v10(msg)
                armed = bool(
                    msg.base_mode
                    & MAV.MAV_MODE_FLAG_SAFETY_ARMED
                )

                if armed or mode != "MANUAL":
                    print("ABORT: Vehicle state changed")
                    return

            master.mav.manual_control_send(
                sysid,
                0,         # x
                STEERING,  # y
                THROTTLE,  # z
                0,         # r
                0          # buttons
            )
            count += 1
            time.sleep(0.1)

        print("Messages sent:", count)
        print("Waiting for RC override timeout...")
        time.sleep(4)
        print("Test finished")

    finally:
        master.close()


if __name__ == "__main__":
    main()
