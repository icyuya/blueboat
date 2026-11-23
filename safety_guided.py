
import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

from pymavlink import mavutil

MAV = mavutil.mavlink

# BlueOSのPython専用Endpoint
CONNECTION = "udpin:0.0.0.0:14551"

# 初期試験用の仮判定値（実機試験前に再評価）
HEARTBEAT_TIMEOUT = 3.0
DATA_TIMEOUT = 3.0
MIN_SATELLITES = 8
MAX_HDOP = 2.0

# MAVLink EKF_STATUS_FLAGS
EKF_REQUIRED = 1 | 2 | 16 | 32
EKF_BAD = 128 | 1024 | 32768

GPS_BIT = MAV.MAV_SYS_STATUS_SENSOR_GPS
AHRS_BIT = MAV.MAV_SYS_STATUS_AHRS

WATCH = {
    "HEARTBEAT",
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "SYS_STATUS",
    "EKF_STATUS_REPORT",
}


def initial_data():
    return {
        "mode": None,
        "armed": False,
        "hb_time": -1e10,
        "gps_time": -1e10,
        "pos_time": -1e10,
        "ekf_time": -1e10,
        "sys_time": -1e10,
        "fix": 0,
        "satellites": 0,
        "hdop": None,
        "lat": None,
        "lon": None,
        "ekf_flags": 0,
        "present": 0,
        "enabled": 0,
        "health": 0,
    }


def safety_check(data, now):
    reasons = []

    if now - data["hb_time"] > HEARTBEAT_TIMEOUT:
        reasons.append("MAVLink Heartbeat timeout")

    if data["mode"] != "GUIDED":
        reasons.append("Not in GUIDED mode")

    if not data["armed"]:
        reasons.append("Vehicle is DISARMED")

    if now - data["gps_time"] > DATA_TIMEOUT:
        reasons.append("GPS data timeout")
    else:
        if data["fix"] < 3:
            reasons.append("GPS has no 3D Fix")
        if (
            data["satellites"] == 255
            or data["satellites"] < MIN_SATELLITES
        ):
            reasons.append("Insufficient satellites")
        if (
            data["hdop"] is None
            or data["hdop"] > MAX_HDOP
        ):
            reasons.append("HDOP invalid or too high")

    if now - data["pos_time"] > DATA_TIMEOUT:
        reasons.append("Position data timeout")
    elif (
        data["lat"] is None
        or data["lon"] is None
        or (data["lat"] == 0 and data["lon"] == 0)
    ):
        reasons.append("Position invalid")

    if now - data["ekf_time"] > DATA_TIMEOUT:
        reasons.append("EKF report timeout")
    else:
        flags = data["ekf_flags"]
        if (flags & EKF_REQUIRED) != EKF_REQUIRED:
            reasons.append("EKF estimate not ready")
        if flags & EKF_BAD:
            reasons.append("EKF warning flags detected")

    if now - data["sys_time"] > DATA_TIMEOUT:
        reasons.append("System status timeout")
    else:
        for name, bit in (
            ("GPS", GPS_BIT),
            ("AHRS", AHRS_BIT),
        ):
            ok = all(
                data[key] & bit
                for key in ("present", "enabled", "health")
            )
            if not ok:
                reasons.append(f"{name} health not OK")

    return reasons


class SafetyController:
    def __init__(self):
        self.state = "STANDBY"
        self.stop_reason = None

    def update(self, data, now, operator_enable=False):
        # 一度介入状態に入ったら復帰しない
        if self.state == "INTERVENTION":
            return

        reasons = safety_check(data, now)

        if self.state == "AUTO_CONTROL":
            if reasons:
                self.state = "INTERVENTION"
                self.stop_reason = reasons
                print("INTERVENTION:", ", ".join(reasons))
                print("Autonomous command generation DISABLED")

        elif self.state == "STANDBY":
            if operator_enable and not reasons:
                self.state = "AUTO_CONTROL"
                print("AUTO_CONTROL enabled (SIMULATION ONLY)")


def guided_command_preview(lat, lon):
    # MAVLinkパケットの内容を辞書で表現するだけ。
    # mav.send() は絶対に呼び出さない。
    return {
        "message": "SET_POSITION_TARGET_GLOBAL_INT",
        "coordinate_frame": 6,
        "type_mask": 3580,
        "lat_int": round(lat * 1e7),
        "lon_int": round(lon * 1e7),
        "alt": 0.0,
    }


def simulate():
    print("=== OFFLINE SAFETY SIMULATION ===")
    print("No connection to BlueBoat\n")

    now = time.monotonic()
    data = initial_data()

    # 以下は架空の試験データ。実機の状態ではない。
    data.update({
        "mode": "GUIDED",
        "armed": True,
        "hb_time": now,
        "gps_time": now,
        "pos_time": now,
        "ekf_time": now,
        "sys_time": now,
        "fix": 3,
        "satellites": 18,
        "hdop": 0.9,
        "lat": 26.0,
        "lon": 127.0,
        "ekf_flags": 831,
        "present": GPS_BIT | AHRS_BIT,
        "enabled": GPS_BIT | AHRS_BIT,
        "health": GPS_BIT | AHRS_BIT,
    })

    base = data.copy()
    controller = SafetyController()

    print("[TEST 1] Operator enables simulated control")
    controller.update(data, now, operator_enable=True)
    assert controller.state == "AUTO_CONTROL"

    print("[TEST 2] Guided target preview")
    command = guided_command_preview(
        26.0000200, 127.0000000
    )
    print(json.dumps(command, indent=2))

    print("[TEST 3] QGC changes mode to MANUAL")
    data["mode"] = "MANUAL"
    controller.update(data, now)
    assert controller.state == "INTERVENTION"

    print("[TEST 4] Mode returns to GUIDED")
    data["mode"] = "GUIDED"
    controller.update(data, now, operator_enable=True)
    assert controller.state == "INTERVENTION"
    print("Automatic restart blocked")

    for label, changes in [
        ("GPS loss", {"fix": 1}),
        ("MAVLink timeout", {"hb_time": now - 5}),
        ("EKF loss", {"ekf_flags": 0}),
    ]:
        print(f"[TEST] {label}")
        test_data = base.copy()
        test_controller = SafetyController()
        test_controller.update(
            test_data, now, operator_enable=True
        )
        test_data.update(changes)
        test_controller.update(test_data, now)
        assert test_controller.state == "INTERVENTION"

    print("\nALL SIMULATION TESTS PASSED")
    print("No MAVLink command was transmitted.")


def live_monitor():
    print("=== BLUEBOAT LIVE MONITOR ===")
    print("RECEIVE ONLY / NO ARM / NO GUIDED COMMANDS")
    print("Listening:", CONNECTION)

    Path("logs").mkdir(exist_ok=True)
    filename = datetime.now().strftime(
        "logs/safety_%Y%m%d_%H%M%S.jsonl"
    )

    master = mavutil.mavlink_connection(CONNECTION)
    data = initial_data()

    system_id = None
    start = time.monotonic()
    last_print = -1e10

    try:
        with open(filename, "w", encoding="utf-8") as log:
            while time.monotonic() - start < 45:
                msg = master.recv_match(
                    blocking=True, timeout=1
                )
                now = time.monotonic()

                if msg is not None:
                    kind = msg.get_type()

                    if kind == "HEARTBEAT":
                        if msg.autopilot != (
                            MAV.MAV_AUTOPILOT_ARDUPILOTMEGA
                        ):
                            continue

                        if system_id is None:
                            system_id = msg.get_srcSystem()
                            print("ArduPilot System:", system_id)

                    if system_id is None:
                        continue

                    if msg.get_srcSystem() != system_id:
                        continue

                    if kind in WATCH:
                        record = {
                            "utc": datetime.now(
                                timezone.utc
                            ).isoformat(),
                            "type": kind,
                            "data": msg.to_dict(),
                        }
                        log.write(json.dumps(record) + "\n")

                    if kind == "HEARTBEAT":
                        data["hb_time"] = now
                        data["mode"] = mavutil.mode_string_v10(msg)
                        data["armed"] = bool(
                            msg.base_mode
                            & MAV.MAV_MODE_FLAG_SAFETY_ARMED
                        )

                    elif kind == "GPS_RAW_INT":
                        data["gps_time"] = now
                        data["fix"] = msg.fix_type
                        data["satellites"] = msg.satellites_visible
                        data["hdop"] = (
                            None if msg.eph == 65535
                            else msg.eph / 100
                        )

                    elif kind == "GLOBAL_POSITION_INT":
                        data["pos_time"] = now
                        data["lat"] = msg.lat / 1e7
                        data["lon"] = msg.lon / 1e7

                    elif kind == "EKF_STATUS_REPORT":
                        data["ekf_time"] = now
                        data["ekf_flags"] = msg.flags

                    elif kind == "SYS_STATUS":
                        data["sys_time"] = now
                        data["present"] = (
                            msg.onboard_control_sensors_present
                        )
                        data["enabled"] = (
                            msg.onboard_control_sensors_enabled
                        )
                        data["health"] = (
                            msg.onboard_control_sensors_health
                        )

                if system_id is not None:
                    if data["armed"]:
                        print("WARNING: ARMED - monitor ending")
                        break

                    if now - data["hb_time"] > HEARTBEAT_TIMEOUT:
                        print("WARNING: MAVLink timeout")
                        break

                if now - last_print >= 5:
                    last_print = now
                    reasons = safety_check(data, now)

                    print(
                        f"\nMode={data['mode']} "
                        f"Armed={data['armed']} "
                        f"GPS_Fix={data['fix']} "
                        f"Sat={data['satellites']} "
                        f"EKF_Flags={data['ekf_flags']}"
                    )

                    if reasons:
                        print("DRY_RUN: BLOCKED")
                        for reason in reasons:
                            print(" -", reason)
                    else:
                        print("Prerequisites: PASS")
                        print("Command transmission remains DISABLED")

    except KeyboardInterrupt:
        print("Monitoring interrupted")
    finally:
        master.close()

    print("Log:", filename)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "mode", choices=["simulate", "live"]
    )
    args = parser.parse_args()

    if args.mode == "simulate":
        simulate()
    else:
        live_monitor()
