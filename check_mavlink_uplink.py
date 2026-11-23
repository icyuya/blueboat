import time
from pymavlink import mavutil

LINK = "udpin:0.0.0.0:14551"
MAV = mavutil.mavlink
PARAM = "RC_OVERRIDE_TIME"

master = mavutil.mavlink_connection(
    LINK,
    source_system=252,
    source_component=190,
)

try:
    print("Waiting for ArduRover heartbeat...")

    sysid = None
    compid = None
    deadline = time.monotonic() + 10

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue
        if msg.autopilot != MAV.MAV_AUTOPILOT_ARDUPILOTMEGA:
            continue

        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()
        armed = bool(
            msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED
        )

        print("System:", sysid, "Component:", compid)
        print("Armed:", armed)

        if armed:
            print("BLOCKED: Please DISARM")
            sysid = None

        break

    if sysid is None:
        print("TEST FAILED: No usable heartbeat")
    else:
        found = False

        for attempt in range(1, 4):
            print(f"Sending PARAM_REQUEST_READ ({attempt}/3)")

            master.mav.param_request_read_send(
                sysid,
                compid,
                PARAM.encode("ascii"),
                -1,
            )

            end = time.monotonic() + 2

            while time.monotonic() < end:
                msg = master.recv_match(
                    blocking=True,
                    timeout=0.5,
                )

                if msg is None:
                    continue
                if msg.get_srcSystem() != sysid:
                    continue

                if msg.get_type() == "HEARTBEAT":
                    armed = bool(
                        msg.base_mode
                        & MAV.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    if armed:
                        print("ABORT: ARMED detected")
                        break

                if msg.get_type() != "PARAM_VALUE":
                    continue

                name = msg.param_id
                if isinstance(name, bytes):
                    name = name.decode("ascii", errors="ignore")

                name = name.strip("\x00")

                if name == PARAM:
                    print("RESPONSE RECEIVED")
                    print("Parameter:", name)
                    print("Value:", msg.param_value)
                    found = True
                    break

            if found or armed:
                break

        if found:
            print("RESULT: MAVLink request/response confirmed")
        else:
            print("RESULT: No matching response received")
finally:
    master.close()
