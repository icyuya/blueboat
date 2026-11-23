
from pymavlink import mavutil
from datetime import datetime, timezone
from pathlib import Path
import time
import math
import json

# BlueOSの既存Endpointを利用
CONNECTION = "udpin:0.0.0.0:14550"

MESSAGE_TYPES = [
    "HEARTBEAT",
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "ATTITUDE",
    "SYS_STATUS",
]

Path("logs").mkdir(exist_ok=True)

filename = datetime.now().strftime(
    "logs/telemetry_%Y%m%d_%H%M%S.jsonl"
)

print("BlueBoatテレメトリ受信開始")
print("接続:", CONNECTION)
print("保存先:", filename)

master = mavutil.mavlink_connection(CONNECTION)

start = time.monotonic()
last_print = {}
received = set()
target_system = None

try:
    with open(filename, "w", encoding="utf-8") as log:

        while time.monotonic() - start < 40:
            msg = master.recv_match(
                type=MESSAGE_TYPES,
                blocking=True,
                timeout=1
            )

            if msg is None:
                continue

            kind = msg.get_type()

            if target_system is None:
                if kind != "HEARTBEAT":
                    continue

                if msg.autopilot != (
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                ):
                    continue

                target_system = msg.get_srcSystem()
                print("ArduPilot System ID:", target_system)

            if msg.get_srcSystem() != target_system:
                continue

            # Heartbeatはフライトコントローラ自身に限定
            if kind == "HEARTBEAT":
                if msg.autopilot != (
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                ):
                    continue

            received.add(kind)

            # 受信メッセージをすべて記録
            data = {
                "received_utc": datetime.now(
                    timezone.utc
                ).isoformat(),
                "type": kind,
                "system_id": msg.get_srcSystem(),
                "component_id": msg.get_srcComponent(),
                "message": msg.to_dict()
            }
            log.write(json.dumps(data) + "\n")

            # 画面表示は種類ごとに3秒間隔
            now = time.monotonic()

            if now - last_print.get(kind, -100) < 3:
                continue

            last_print[kind] = now

            if kind == "HEARTBEAT":
                armed = bool(
                    msg.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

                print(
                    f"[MODE] {mavutil.mode_string_v10(msg)} "
                    f"ARMED={armed}"
                )

                if armed:
                    print("WARNING: ARM状態を検出しました")

            elif kind == "GPS_RAW_INT":
                hdop = (
                    msg.eph / 100
                    if msg.eph != 65535 else None
                )

                print(
                    f"[GPS] FIX={msg.fix_type} "
                    f"衛星数={msg.satellites_visible} "
                    f"HDOP={hdop}"
                )

                if msg.fix_type >= 2:
                    print(
                        f"  緯度={msg.lat / 1e7:.7f} "
                        f"経度={msg.lon / 1e7:.7f}"
                    )

            elif kind == "GLOBAL_POSITION_INT":
                speed = math.hypot(
                    msg.vx, msg.vy
                ) / 100

                print(
                    f"[POSITION] "
                    f"緯度={msg.lat / 1e7:.7f} "
                    f"経度={msg.lon / 1e7:.7f} "
                    f"速度={speed:.2f}m/s"
                )

            elif kind == "ATTITUDE":
                print(
                    f"[IMU/ATTITUDE] "
                    f"Roll={math.degrees(msg.roll):.1f}° "
                    f"Pitch={math.degrees(msg.pitch):.1f}° "
                    f"Yaw={math.degrees(msg.yaw) % 360:.1f}°"
                )

            elif kind == "SYS_STATUS":
                voltage = (
                    msg.voltage_battery / 1000
                    if msg.voltage_battery != 65535
                    else None
                )

                remaining = (
                    msg.battery_remaining
                    if msg.battery_remaining >= 0
                    else None
                )

                print(
                    f"[BATTERY] "
                    f"電圧={voltage}V "
                    f"残量={remaining}%"
                )

except KeyboardInterrupt:
    print("受信を終了しました")

finally:
    master.close()

print("\n--- 受信結果 ---")
for kind in MESSAGE_TYPES:
    result = "OK" if kind in received else "未受信"
    print(f"{kind}: {result}")

print("ログ保存先:", filename)
