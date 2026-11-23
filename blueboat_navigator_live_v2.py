
import socket
import time

from pymavlink import mavutil

from blueboat_navigator import (
    distance_bearing,
    destination,
    parse_target_packet,
    create_guided,
    STANDOFF_M,
    TARGET_RECV_BUFFER_BYTES,
)

MAVLINK = "udpin:0.0.0.0:14551"
HOST = "127.0.0.1"
PORT = 16000

BOAT_SYSID = 1
BOAT_COMPID = 1

HEARTBEAT_TIMEOUT = 3.0
GPS_TIMEOUT = 3.0
MIN_SATELLITES = 8
MAX_HDOP = 2.0
MAX_TARGET_DISTANCE = 100.0

# DRY RUN中のみ許容する初期モード
SAFE_MODES = ("MANUAL", "HOLD")


def main():
    print("=== BlueBoat Live Navigator V2 ===")
    print("READ ONLY / GUIDED DRY RUN")
    print("No ARM or movement commands")

    boat = {
        "lat": None,
        "lon": None,
        "fix": 0,
        "sat": 0,
        "hdop": None,
        "gps_time": 0.0,
        "pos_time": 0.0,
        "hb_time": None,
        "mode": None,
        "armed": None,
        "latched": False,
        "latch_reason": None,
    }

    master = mavutil.mavlink_connection(MAVLINK)

    def latch(reason):
        if not boat["latched"]:
            boat["latched"] = True
            boat["latch_reason"] = reason
            print("SAFETY LATCHED:", reason)

    def read_telemetry():
        for _ in range(200):
            msg = master.recv_match(blocking=False)

            if msg is None:
                break

            kind = msg.get_type()
            now = time.monotonic()

            if kind == "HEARTBEAT":
                if (
                    msg.get_srcSystem() != BOAT_SYSID
                    or msg.get_srcComponent() != BOAT_COMPID
                    or msg.type !=
                    mavutil.mavlink.MAV_TYPE_SURFACE_BOAT
                    or msg.autopilot !=
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                ):
                    continue

                previous = boat["hb_time"]

                if previous is not None:
                    if now - previous > HEARTBEAT_TIMEOUT:
                        latch("Heartbeat timeout")

                boat["hb_time"] = now
                boat["mode"] = mavutil.mode_string_v10(msg)
                boat["armed"] = bool(
                    msg.base_mode &
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

                if boat["mode"] not in SAFE_MODES:
                    latch("Unexpected mode: " + boat["mode"])

                if boat["armed"]:
                    latch("Vehicle armed during DRY RUN")

                continue

            if (
                msg.get_srcSystem() != BOAT_SYSID
                or msg.get_srcComponent() != BOAT_COMPID
            ):
                continue

            if kind == "GPS_RAW_INT":
                boat["fix"] = msg.fix_type
                boat["sat"] = msg.satellites_visible
                boat["hdop"] = (
                    None if msg.eph == 65535
                    else msg.eph / 100.0
                )
                boat["gps_time"] = now

            elif kind == "GLOBAL_POSITION_INT":
                boat["lat"] = msg.lat / 1e7
                boat["lon"] = msg.lon / 1e7
                boat["pos_time"] = now

    def safety_reason():
        now = time.monotonic()

        if boat["hb_time"] is None:
            return "No ArduRover heartbeat"

        if now - boat["hb_time"] > HEARTBEAT_TIMEOUT:
            latch("Heartbeat timeout")

        if boat["mode"] not in SAFE_MODES:
            latch("Unexpected mode")

        if boat["armed"]:
            latch("Vehicle armed during DRY RUN")

        if boat["latched"]:
            return (
                "SAFETY LATCHED: " +
                str(boat["latch_reason"])
            )

        return None

    def current_position():
        now = time.monotonic()

        if now - boat["gps_time"] > GPS_TIMEOUT:
            return None, "GPS data unavailable"

        if boat["fix"] < 3:
            return None, (
                f"No 3D GPS Fix (fix={boat['fix']})"
            )

        if (
            boat["sat"] == 255
            or boat["sat"] < MIN_SATELLITES
        ):
            return None, "Insufficient GPS satellites"

        if (
            boat["hdop"] is None
            or boat["hdop"] > MAX_HDOP
        ):
            return None, "GPS HDOP too high or unknown"

        if now - boat["pos_time"] > GPS_TIMEOUT:
            return None, "Position data unavailable"

        lat = boat["lat"]
        lon = boat["lon"]

        if lat is None or lon is None:
            return None, "Position unknown"

        if lat == 0 and lon == 0:
            return None, "Position invalid"

        return (lat, lon), "OK"

    try:
        with socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        ) as sock:
            sock.bind((HOST, PORT))
            sock.settimeout(0.1)

            print(f"Target UDP: {HOST}:{PORT}")
            print("Press Ctrl+C to stop")

            last_status = None
            last_gps_print = 0.0

            while True:
                read_telemetry()

                reason = safety_reason()

                if boat["hb_time"] is None:
                    status = "WAITING FOR HEARTBEAT"
                elif boat["latched"]:
                    status = "SAFETY LATCHED"
                else:
                    status = "MONITOR OK"

                if status != last_status:
                    print("\nStatus:", status)
                    print("Mode:", boat["mode"])
                    print("Armed:", boat["armed"])
                    last_status = status

                now = time.monotonic()

                if now - last_gps_print >= 5:
                    last_gps_print = now

                    position, gps_reason = current_position()

                    print(
                        f"\nGPS Fix={boat['fix']} "
                        f"Sat={boat['sat']} "
                        f"HDOP={boat['hdop']}"
                    )

                    if position is None:
                        print("POSITION BLOCKED:", gps_reason)
                    else:
                        print(
                            "BlueBoat:",
                            f"{position[0]:.7f},",
                            f"{position[1]:.7f}"
                        )

                try:
                    raw, address = sock.recvfrom(TARGET_RECV_BUFFER_BYTES)
                except socket.timeout:
                    continue

                try:
                    target = parse_target_packet(raw)
                except ValueError as exc:
                    print("TARGET REJECTED:", exc)
                    continue

                print("\nTARGET RECEIVED")

                # 安全監視を最優先
                reason = safety_reason()

                if reason is not None:
                    print("GUIDED BLOCKED:", reason)
                    continue

                position, gps_reason = current_position()

                if position is None:
                    print("GUIDED BLOCKED:", gps_reason)
                    continue

                lat, lon = position

                distance, bearing = distance_bearing(
                    lat,
                    lon,
                    target["lat"],
                    target["lon"],
                )

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
