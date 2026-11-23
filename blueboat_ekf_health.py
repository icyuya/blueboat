import logging
import math
from dataclasses import dataclass

from pymavlink.dialects.v20 import ardupilotmega as mav


LOGGER = logging.getLogger(__name__)
EKF_MAX_AGE_S = 3.0
EKF_BLOCK_FLAGS = (
    (mav.EKF_CONST_POS_MODE, "EKF_CONST_POS_MODE"),
    (mav.EKF_UNINITIALIZED, "EKF_UNINITIALIZED"),
    (mav.EKF_GPS_GLITCHING, "EKF_GPS_GLITCHING"),
)
VARIANCE_FIELDS = (
    "velocity_variance",
    "pos_horiz_variance",
    "pos_vert_variance",
    "compass_variance",
    "terrain_alt_variance",
    "airspeed_variance",
)


@dataclass(frozen=True)
class EkfReport:
    received_at: float
    flags: int | None


def ekf_block_reasons(report, *, monotonic_now, max_age_s=EKF_MAX_AGE_S):
    if not math.isfinite(max_age_s) or max_age_s <= 0:
        raise ValueError("EKF max_age_s must be finite and positive")
    if report is None:
        return ("EKF_STATUS_REPORT not received",)

    age = monotonic_now - report.received_at
    if not math.isfinite(age) or age < 0:
        return ("Invalid EKF reception time",)
    if age >= max_age_s:
        return ("EKF_STATUS_REPORT expired",)

    flags = report.flags
    if type(flags) is not int or not 0 <= flags <= 0xFFFF:
        return ("Invalid EKF_STATUS_REPORT flags",)

    reasons = [f"{name} set" for bit, name in EKF_BLOCK_FLAGS if flags & bit]
    if not flags & mav.EKF_POS_HORIZ_ABS:
        reasons.append("EKF_POS_HORIZ_ABS not set")
    return tuple(reasons)


class EkfHealthMonitor:
    def __init__(self, *, system_id, component_id, max_age_s=EKF_MAX_AGE_S):
        self._source = (system_id, component_id)
        self._max_age_s = max_age_s
        self._report = None
        self._last_reasons = None

    def observe(self, message, *, received_at):
        if message.get_type() != "EKF_STATUS_REPORT":
            return False
        source = (message.get_srcSystem(), message.get_srcComponent())
        if source != self._source:
            return False

        self._report = EkfReport(received_at, getattr(message, "flags", None))
        if LOGGER.isEnabledFor(logging.DEBUG):
            variances = {field: getattr(message, field, None) for field in VARIANCE_FIELDS}
            LOGGER.debug(
                "EKF_STATUS_REPORT source=%s received_monotonic=%s flags=%r variances=%s",
                source, received_at, self._report.flags, variances,
            )
        return True

    def block_reasons(self, *, monotonic_now, gnss_reason):
        ekf_reasons = ekf_block_reasons(
            self._report, monotonic_now=monotonic_now, max_age_s=self._max_age_s,
        )
        reasons = ekf_reasons if gnss_reason is None else (gnss_reason, *ekf_reasons)
        if reasons and reasons != self._last_reasons:
            LOGGER.warning("GUIDED BLOCKED: %s", "; ".join(reasons))
        self._last_reasons = reasons
        return reasons
