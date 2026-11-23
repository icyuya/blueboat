
import socket
import json
import time

from pymavlink import mavutil

from blueboat_navigator import (
    distance_bearing,
    destination,
    validate,
    create_guided,
    STANDOFF_M,
)

MAVLINK = "udpin:0.0.0.0:14551"
HOST = "127.0.0.1"
PORT = 16000

GPS_TIMEOUT = 3.0
MIN_SATELLITES = 8
MAX_HDOP = 2.0
MAX_TARGET_DISTANCE = 100.0


def main():
    print("=== BlueBoat Live Navigator ===")
    print("READ ONLY / GUIDED DRY RUN")
    print("No ARM or movement commands")

    boat = {
        "sysid": None,
        "lat": None,
        "lon": None,
        "fix": 0,
        "sat": 0,
        "hdop": None,
        "gps_time": 0.0,
        "pos_time": 0.0,
    }

    master = mavutil.mavlink_connection(MAVLINK)

    def read_telemetry():
        for _ in range(200):
            msg = master.recv_match(blocking=False)

            if msg is None:
                break

            kind = msg.get_type()

            if kind == "HEARTBEAT":
                if msg.autopilot != (
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                ):
                    continue

                boat["sysid"] = msg.get_srcSystem()

            if boat["sysid"] is None:
                continue

            if msg.get_srcSystem() != boat["sysid"]:
                continue

            if kind == "GPS_RAW_INT":
                boat["fix"] = msg.fix_type
                boat["sat"] = msg.satellites_visible

                boat["hdop"] = (
                    None if msg.eph == 65535
                    else msg.eph / 100.0
                )

                boat["gps_time"] = time.monotonic()

            elif kind == "GLOBAL_POSITION_INT":
                boat["lat"] = msg.lat / 1e7
                boat["lon"] = msg.lon / 1e7
                boat["pos_time"] = time.monotonic()

    def current_position():
        now = time.monotonic()

        if boat["sysid"] is None:
            return None, "No ArduRover heartbeat"

        if now - boat["gps_time"] > GPS_TIMEOUT:
            return None, "GPS data unavailable"

        if boat["fix"] < 3:
            return None, f"No 3D GPS Fix (fix={boat['fix']})"

        if boat["sat"] == 255 or boat["sat"] < MIN_SATELLITES:
            return None, "Insufficient GPS satellites"

        if boat["hdop"] is None or boat["hdop"] > MAX_HDOP:
            return None, "GPS HDOP too high or unknown"

        if now - boat["pos_time"] > GPS_TIMEOUT:
            return None, "Position data unavailable"

        lat = boat["lat"]
        lon = boat["lon"]

        if lat is None or lon is None:
            return None, "Position unknown"

        if (lat == 0 and lon == 0):
            return None, "Position invalid"

        return (lat, lon), "OK"

    with socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    ) as sock:
        sock.bind((HOST, PORT))
        sock.settimeout(0.1)

        print(f"Target UDP: {HOST}:{PORT}")
        print("Press Ctrl+C to stop")

        last_status = 0.0

        try:
            while True:
                read_telemetry()

                now = time.monotonic()

                if now - last_status >= 5:
                    last_status = now

                    position, reason = current_position()

                    print(
                        f"\nGPS Fix={boat['fix']} "
                        f"Sat={boat['sat']} "
                        f"HDOP={boat['hdop']}"
                    )

                    if position is None:
                        print("POSITION BLOCKED:", reason)
                    else:
                        print(
                            "BlueBoat:",
                            f"{position[0]:.7f},",
                            f"{position[1]:.7f}"
                        )

                try:
                    raw, address = sock.recvfrom(4096)
                except socket.timeout:
                    continue

                try:
                    target = json.loads(raw.decode("utf-8"))
                    validate(target)
                except (ValueError, UnicodeError) as exc:
                    print("TARGET REJECTED:", exc)
                    continue

                position, reason = current_position()

                if position is None:
                    print("TARGET RECEIVED")
                    print("GUIDED BLOCKED:", reason)
                    continue

                lat, lon = position

                distance, bearing = distance_bearing(
                    lat,
                    lon,
                    target["lat"],
                    target["lon"],
                )

                print("\n--- TARGET ---")
                print(f"Distance: {distance:.2f} m")
                print(f"Bearing: {bearing:.1f} deg")

                if distance > MAX_TARGET_DISTANCE:
                    print("BLOCKED: Target too far")
                    continue

                if distance <= STANDOFF_M:
                    print("BLOCKED: Inside standoff distance")
                    continue

                travel = distance - STANDOFF_M

                goal_lat, goal_lon = destination(
                    lat, lon, bearing, travel
                )

                message = create_guided(
                    goal_lat, goal_lon
                )

                print(
                    "Approach waypoint:",
                    f"{goal_lat:.7f}, {goal_lon:.7f}"
                )

                print("GUIDED:", message)
                print("DRY RUN - NOT TRANSMITTED")

        except KeyboardInterrupt:
            print("\nStopped")

        finally:
            master.close()


if __name__ == "__main__":
    main()
