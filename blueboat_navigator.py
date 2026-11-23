
import json
import math
import socket
import time

from pymavlink import mavutil

# BlueBoatの架空の現在位置
BOAT_LAT = 26.0000000
BOAT_LON = 127.0000000

# 仮の接近停止距離（実際の安全距離ではない）
STANDOFF_M = 3.0

HOST = "127.0.0.1"
PORT = 16000
MAX_TARGET_PACKET_BYTES = 4096
TARGET_RECV_BUFFER_BYTES = 65535
EARTH_RADIUS = 6371000.0

MAV = mavutil.mavlink


def distance_bearing(lat1, lon1, lat2, lon2):
    a1 = math.radians(lat1)
    a2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)

    h = (
        math.sin((a2 - a1) / 2) ** 2
        + math.cos(a1) * math.cos(a2)
        * math.sin(dl / 2) ** 2
    )

    distance = 2 * EARTH_RADIUS * math.asin(
        min(1.0, math.sqrt(h))
    )

    y = math.sin(dl) * math.cos(a2)
    x = (
        math.cos(a1) * math.sin(a2)
        - math.sin(a1) * math.cos(a2) * math.cos(dl)
    )

    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

    return distance, bearing


def destination(lat, lon, bearing, distance):
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    theta = math.radians(bearing)
    delta = distance / EARTH_RADIUS

    lat2 = math.asin(
        math.sin(lat1) * math.cos(delta)
        + math.cos(lat1) * math.sin(delta) * math.cos(theta)
    )

    lon2 = lon1 + math.atan2(
        math.sin(theta) * math.sin(delta) * math.cos(lat1),
        math.cos(delta) - math.sin(lat1) * math.sin(lat2),
    )

    return (
        math.degrees(lat2),
        (math.degrees(lon2) + 180) % 360 - 180,
    )


def validate(data, *, now=None):
    if not isinstance(data, dict):
        raise ValueError("Invalid JSON object")

    if data.get("type") != "target":
        raise ValueError("Invalid message type")

    for key in ("lat", "lon", "confidence", "sent_at"):
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"Invalid {key}")
        try:
            finite = math.isfinite(value)
        except OverflowError:
            raise ValueError(f"Invalid {key}") from None
        if not finite:
            raise ValueError(f"Invalid {key}")

    if not -90 <= data["lat"] <= 90:
        raise ValueError("Invalid latitude")

    if not -180 <= data["lon"] <= 180:
        raise ValueError("Invalid longitude")

    if not 0.8 <= data["confidence"] <= 1:
        raise ValueError("Low confidence")

    if now is None:
        now = time.time()
    if abs(now - data["sent_at"]) > 5:
        raise ValueError("Expired timestamp")


def _reject_json_constant(value):
    raise ValueError(f"Invalid JSON constant: {value}")


def _unique_json_object(pairs):
    data = {}
    for key, value in pairs:
        if key in data:
            raise ValueError("Duplicate JSON key")
        data[key] = value
    return data


def parse_target_packet(raw, *, now=None):
    if len(raw) > MAX_TARGET_PACKET_BYTES:
        raise ValueError("Target packet too large")

    try:
        target = json.loads(
            raw.decode("utf-8"),
            parse_constant=_reject_json_constant,
            object_pairs_hook=_unique_json_object,
        )
    except RecursionError:
        raise ValueError("Target JSON nested too deeply") from None

    validate(target, now=now)
    return target


def create_guided(lat, lon):
    return MAV.MAVLink_set_position_target_global_int_message(
        0,
        1,
        1,
        MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        3580,
        round(lat * 1e7),
        round(lon * 1e7),
        0,
        0, 0, 0,
        0, 0, 0,
        0, 0,
    )


def main():
    print("=== BlueBoat Navigator ===")
    print("DRY RUN - NO VEHICLE COMMANDS")
    print(f"Boat: {BOAT_LAT:.7f}, {BOAT_LON:.7f}")
    print(f"Standoff: {STANDOFF_M} m")

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((HOST, PORT))
        print(f"Listening on {HOST}:{PORT}")

        try:
            while True:
                raw, address = sock.recvfrom(TARGET_RECV_BUFFER_BYTES)

                try:
                    target = parse_target_packet(raw)

                    distance, bearing = distance_bearing(
                        BOAT_LAT,
                        BOAT_LON,
                        target["lat"],
                        target["lon"],
                    )

                    print("\n--- NEW TARGET ---")
                    print(f"Target distance: {distance:.2f} m")
                    print(f"Target bearing: {bearing:.1f} deg")

                    if distance <= STANDOFF_M:
                        print("Inside standoff distance")
                        print("No approach waypoint generated")
                        continue

                    travel = distance - STANDOFF_M

                    goal_lat, goal_lon = destination(
                        BOAT_LAT, BOAT_LON, bearing, travel
                    )

                    message = create_guided(goal_lat, goal_lon)

                    print(f"Approach distance: {travel:.2f} m")
                    print(
                        f"Waypoint: "
                        f"{goal_lat:.7f}, {goal_lon:.7f}"
                    )
                    print("GUIDED:", message)
                    print("DRY RUN ONLY")

                except ValueError as exc:
                    print("REJECTED:", exc)

        except KeyboardInterrupt:
            print("\nStopped")


if __name__ == "__main__":
    main()
