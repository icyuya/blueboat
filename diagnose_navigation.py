
from pymavlink import mavutil
from datetime import datetime, timezone
from pathlib import Path
import json
import time

CONNECTION = "udpin:0.0.0.0:14550"
DURATION = 120

mav = mavutil.mavlink
SENSORS = {
    "GPS": mav.MAV_SYS_STATUS_SENSOR_GPS,
    "GYRO": mav.MAV_SYS_STATUS_SENSOR_3D_GYRO,
    "ACCEL": mav.MAV_SYS_STATUS_SENSOR_3D_ACCEL,
    "COMPASS": mav.MAV_SYS_STATUS_SENSOR_3D_MAG,
    "AHRS": mav.MAV_SYS_STATUS_AHRS,
}

WATCH = {
    "HEARTBEAT",
    "GPS_RAW_INT",
    "SYS_STATUS",
    "EKF_STATUS_REPORT",
    "STATUSTEXT",
}

Path("logs").mkdir(exist_ok=True)
filename = datetime.now().strftime(
    "logs/navigation_%Y%m%d_%H%M%S.jsonl"
)

master = mavutil.mavlink_connection(CONNECTION)

print("Navigation diagnostics")
print("Connection:", CONNECTION)
print("Log:", filename)

start = time.monotonic()
last_print = {}
received = set()
target = None

try:
    with open(filename, "w", encoding="utf-8") as log:
        while time.monotonic() - start < DURATION:
            msg = master.recv_match(
                blocking=True, timeout=1
            )

            if msg is None:
                continue

            kind = msg.get_type()

            if target is None:
                if kind != "HEARTBEAT":
                    continue
                if msg.autopilot != mav.MAV_AUTOPILOT_ARDUPILOTMEGA:
                    continue
                target = msg.get_srcSystem()
                print("ArduPilot system:", target)

            if msg.get_srcSystem() != target:
                continue

            if kind not in WATCH:
                continue

            received.add(kind)

            entry = {
                "utc": datetime.now(timezone.utc).isoformat(),
                "type": kind,
                "data": msg.to_dict()
            }
            log.write(json.dumps(entry, default=str) + "\n")

            if kind == "STATUSTEXT":
                print("[MESSAGE]", msg.text)
                continue

            now = time.monotonic()
            if now - last_print.get(kind, -100) < 10:
                continue
            last_print[kind] = now

            if kind == "HEARTBEAT":
                armed = bool(
                    msg.base_mode &
                    mav.MAV_MODE_FLAG_SAFETY_ARMED
                )
                print(
                    "[MODE]",
                    mavutil.mode_string_v10(msg),
                    "Armed:", armed
                )

            elif kind == "GPS_RAW_INT":
                hdop = (
                    msg.eph / 100
                    if msg.eph != 65535 else None
                )
                print(
                    f"[GPS] Fix={msg.fix_type} "
                    f"Sat={msg.satellites_visible} "
                    f"HDOP={hdop}"
                )

            elif kind == "SYS_STATUS":
                present = msg.onboard_control_sensors_present
                enabled = msg.onboard_control_sensors_enabled
                health = msg.onboard_control_sensors_health

                print("[SENSOR STATUS]")
                for name, bit in SENSORS.items():
                    print(
                        f"  {name}: "
                        f"present={bool(present & bit)} "
                        f"enabled={bool(enabled & bit)} "
                        f"healthy={bool(health & bit)}"
                    )

            elif kind == "EKF_STATUS_REPORT":
                print(
                    "[EKF]",
                    "flags=", msg.flags,
                    "pos_var=", msg.pos_horiz_variance,
                    "vel_var=", msg.velocity_variance,
                    "compass_var=", msg.compass_variance
                )

finally:
    master.close()

print("\n--- Summary ---")
for kind in sorted(WATCH):
    print(kind, "OK" if kind in received else "未受信")
print("Log:", filename)
