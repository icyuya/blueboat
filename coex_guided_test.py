
import argparse
import json
import math
import socket
import time

from pymavlink import mavutil

HOST = "127.0.0.1"
PORT = 16000

MAV = mavutil.mavlink


def validate_target(data):
    if not isinstance(data, dict):
        raise ValueError("JSON must be an object")

    if data.get("type") != "target":
        raise ValueError("Unknown message type")

    for key in ("lat", "lon", "confidence", "sent_at"):
        value = data.get(key)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ValueError(f"Invalid {key}")

    lat = data["lat"]
    lon = data["lon"]

    if not -90 <= lat <= 90:
        raise ValueError("Latitude out of range")

    if not -180 <= lon <= 180:
        raise ValueError("Longitude out of range")

    if not 0 <= data["confidence"] <= 1:
        raise ValueError("Confidence out of range")

    # This demonstration runs on one PC,
    # so sender and receiver share a clock.
    age = time.time() - data["sent_at"]

    if abs(age) > 5:
        raise ValueError("Stale or invalid timestamp")

    if data["confidence"] < 0.8:
        raise ValueError("Confidence too low")

    return lat, lon


def create_guided_message(lat, lon):
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


def receive_targets():
    with socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    ) as sock:

        sock.bind((HOST, PORT))

        print("=== BlueBoat Target Receiver ===")
        print(f"Listening: {HOST}:{PORT}")
        print("DRY RUN: No MAVLink commands transmitted")
        print("Press Ctrl+C to stop\n")

        try:
            while True:
                raw, address = sock.recvfrom(4096)

                try:
                    data = json.loads(raw.decode("utf-8"))
                    lat, lon = validate_target(data)

                    message = create_guided_message(
                        lat, lon
                    )

                    print("\nTARGET RECEIVED")
                    print(f"From: {address}")
                    print(f"Latitude:  {lat:.7f}")
                    print(f"Longitude: {lon:.7f}")
                    print(
                        "Confidence:",
                        data["confidence"]
                    )
                    print("GUIDED message:")
                    print(message)
                    print("DRY RUN ONLY")

                except (ValueError, UnicodeError) as exc:
                    print("TARGET REJECTED:", exc)

        except KeyboardInterrupt:
            print("\nReceiver stopped")


def send_demo():
    target = {
        "type": "target",
        "lat": 26.0000449,
        "lon": 127.0000000,
        "confidence": 0.95,
        "sent_at": time.time(),
    }

    packet = json.dumps(target).encode("utf-8")

    with socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    ) as sock:
        sock.sendto(packet, (HOST, PORT))

    print("Simulated COEX target transmitted")
    print(json.dumps(target, indent=2))


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "mode",
        choices=["receive", "send-demo"]
    )

    args = parser.parse_args()

    if args.mode == "receive":
        receive_targets()
    else:
        send_demo()


if __name__ == "__main__":
    main()
