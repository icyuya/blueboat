
import math
from pymavlink import mavutil

# ===================================
# BlueBoat GUIDED Control - DRY RUN
# ===================================

MAV = mavutil.mavlink

# テスト用の架空座標
CURRENT_LAT = 26.0000000
CURRENT_LON = 127.0000000

# 現在地から北へ5m
DISTANCE_M = 5.0

# 目標座標を計算
TARGET_LAT = CURRENT_LAT + DISTANCE_M / 111320
TARGET_LON = CURRENT_LON

# GUIDED位置制御用メッセージ
message = MAV.MAVLink_set_position_target_global_int_message(
    0,                                  # time_boot_ms
    1,                                  # target_system
    1,                                  # target_component
    MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    3580,                               # Position only
    int(round(TARGET_LAT * 1e7)),
    int(round(TARGET_LON * 1e7)),
    0,                                  # altitude (ignored)
    0, 0, 0,                            # velocity
    0, 0, 0,                            # acceleration
    0,                                  # yaw
    0                                   # yaw_rate
)

# 距離と方位を計算
lat1 = math.radians(CURRENT_LAT)
lat2 = math.radians(TARGET_LAT)
dlon = math.radians(TARGET_LON - CURRENT_LON)

a = (
    math.sin((lat2 - lat1) / 2) ** 2
    + math.cos(lat1)
    * math.cos(lat2)
    * math.sin(dlon / 2) ** 2
)

distance = 6371000 * 2 * math.atan2(
    math.sqrt(a), math.sqrt(1 - a)
)

print("=== BlueBoat GUIDED DRY RUN ===")
print(f"Current: {CURRENT_LAT:.7f}, {CURRENT_LON:.7f}")
print(f"Target:  {TARGET_LAT:.7f}, {TARGET_LON:.7f}")
print(f"Distance: {distance:.2f} m")
print(f"Direction: North")
print()
print("MAVLink Message:")
print(message)
print()
print("DRY RUN: No commands transmitted")
