"""Maxon EPOS4 / IDX drives (including the HEJ gearhead joints).

Differences from eRob that matter here:

* The encoder sits on the *motor* shaft, before the gearbox, so `gear_ratio`
  in the axis configuration must be the real reduction (e.g. 100 for a 100:1
  HEJ). `counts_per_rev` is the encoder's counts per motor revolution, which
  is 4x the line count for a quadrature encoder.
* EPOS4 enforces a following-error window (0x6065) and trips on violation.
  The window is set from the axis limits so the drive and the master agree.
* Modes of operation must be selected before OP if the drive is configured
  for a fixed mode; mapping 0x6060 into the RxPDO (the default here) makes it
  switchable at runtime.
"""
from __future__ import annotations

import logging
import struct

import pysoem

from .. import cia402 as c
from .base import Driver, register

log = logging.getLogger(__name__)

#: Maxon's EtherCAT vendor ID.
MAXON_VENDOR_ID = 0x000000FB

# --- Maxon-specific objects ---------------------------------------------
OD_FOLLOWING_ERROR_WINDOW = 0x6065
OD_FOLLOWING_ERROR_TIMEOUT = 0x6066
OD_POSITION_WINDOW = 0x6067
OD_POSITION_WINDOW_TIME = 0x6068
OD_MOTOR_RATED_CURRENT = 0x6075
OD_MAX_MOTOR_SPEED = 0x6080
OD_INTERPOLATION_TIME = 0x60C2
#: EPOS4 axis configuration; writing it requires the drive to be in PRE-OP.
OD_MAXON_AXIS_CONFIG = 0x3000


@register
class MaxonDriver(Driver):
    key = "maxon"
    display_name = "Maxon EPOS4/IDX"
    vendor_id = MAXON_VENDOR_ID

    def configure_extra(self, slave: pysoem.CdefSlave) -> None:
        """Set the interpolation period and the drive-side error windows."""
        self._write_interpolation_period(slave)
        self._write_error_windows(slave)

    def _write_interpolation_period(self, slave: pysoem.CdefSlave) -> None:
        already_set = any(
            (int(i["index"], 0) if isinstance(i["index"], str) else i["index"])
            == OD_INTERPOLATION_TIME
            for i in self.cfg.startup_sdo
        )
        if already_set:
            return
        try:
            slave.sdo_write(OD_INTERPOLATION_TIME, 1, struct.pack("<B", 2))
            slave.sdo_write(OD_INTERPOLATION_TIME, 2, struct.pack("<b", -3))
        except Exception:  # noqa: BLE001 - optional on some firmware
            log.debug("%s: 0x60C2 not writable", self.cfg.name)

    def _write_error_windows(self, slave: pysoem.CdefSlave) -> None:
        """Mirror the master's following-error limit into the drive.

        The master latches a fault at `max_following_error_deg`; the drive is
        given the same window so it can react even if the master stalls. The
        drive's window is in encoder counts on the motor shaft.
        """
        limits = self.cfg.limits
        window = abs(self.cfg.deg_to_counts_delta(limits.max_following_error_deg))
        if window <= 0:
            return
        try:
            slave.sdo_write(
                OD_FOLLOWING_ERROR_WINDOW, 0, struct.pack("<I", int(window))
            )
            log.debug("%s: following error window = %d counts",
                      self.cfg.name, window)
        except Exception:  # noqa: BLE001
            log.debug("%s: 0x6065 not writable", self.cfg.name)

    def apply_motion_limits(self, slave: pysoem.CdefSlave) -> None:
        """Also cap the motor speed, which EPOS4 enforces independently.

        0x6080 is in rpm at the *motor* shaft, so the output-shaft limit from
        the configuration is multiplied by the gear ratio.
        """
        super().apply_motion_limits(slave)
        output_rpm = self.cfg.limits.max_velocity_deg_s * 60.0 / 360.0
        motor_rpm = int(round(output_rpm * self.cfg.gear_ratio))
        if motor_rpm <= 0:
            return
        try:
            slave.sdo_write(OD_MAX_MOTOR_SPEED, 0, struct.pack("<I", motor_rpm))
            log.info("%s: max motor speed = %d rpm", self.cfg.name, motor_rpm)
        except Exception:  # noqa: BLE001
            log.debug("%s: 0x6080 not writable", self.cfg.name)

    def read_rated_torque_mnm(self, slave: pysoem.CdefSlave) -> float:
        """Read 0x6076 (rated torque, mNm) so per-mille torque can be reported
        in physical units. Returns the configured value if the object is
        missing."""
        try:
            raw = slave.sdo_read(c.OD_MOTOR_RATED_TORQUE, 0)
            return float(struct.unpack("<I", raw[:4])[0])
        except Exception:  # noqa: BLE001
            return self.cfg.rated_torque_mnm
