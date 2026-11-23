
from pymavlink import mavutil
import time

# BlueOSから送られるMAVLinkを待ち受ける
CONNECTION = "udpin:0.0.0.0:14550"

print("BlueBoatとの通信確認を開始します")
print("接続先:", CONNECTION)

master = mavutil.mavlink_connection(CONNECTION)

print("Heartbeatを待っています...")

count = 0
start = time.monotonic()

try:
    while count < 3 and time.monotonic() - start < 30:
        msg = master.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1
        )

        if msg is None:
            continue

        # ArduPilot本体のHeartbeatのみ対象
        if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            continue

        count += 1

        armed = bool(
            msg.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        mode = mavutil.mode_string_v10(msg)

        print(f"\nHeartbeat {count}/3")
        print("System ID:", msg.get_srcSystem())
        print("Component ID:", msg.get_srcComponent())
        print("Mode:", mode)
        print("Armed:", armed)

        if armed:
            print("WARNING: 機体がARM状態です")

    if count == 3:
        print("\n通信確認成功！")
    else:
        print("\nHeartbeatを3回受信できませんでした")

finally:
    master.close()
