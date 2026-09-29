"""Maxon EPOS4 drives, including the HEJ joints (verified on a HEJ 70).

What the HEJ 70 on the bench reports (read over SDO, see configs/demo_1x_hej70.yaml):

* Gear 18:1 (0x3003). Sensors: digital incremental encoder (20480 inc/rev,
  motor side), SSI absolute encoder, Hall sensors; dual-loop control.
* Main position sensor resolution 16384 inc/rev (0x3000:05). This is the SSI
  encoder, not the motor encoder, so position values are OUTPUT-shaft counts:
  configure `counts_per_rev: 16384` with `gear_ratio: 1.0`. The drive's gear
  ratio is still read from 0x3003 to limit motor speed (0x6080).
* Velocity objects use EPOS4 velocity units (0x60A9 = 0.001 rpm), not
  counts/s, so 0x607F is not written and target velocity is not mapped: CSV
  stays refused until the unit/shaft relation is verified on hardware.
* 0x6502 reads 0x624 (no CSP) but the drive accepts PP, PV, HMM, CSP, CSV and
  CST in 0x6060. The object is ignored.
* Factory PDOs map only controlword/statusword; the layout below replaces them.
"""
from __future__ import annotations

import logging
import struct

import pysoem

from .. import cia402 as c
from ..pdo import PdoEntry, PdoMap, pdo_from_config
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
#: Gear configuration: :1 reduction numerator, :2 denominator.
OD_GEAR_CONFIG = 0x3003


@register
class MaxonDriver(Driver):
    key = "maxon"
    display_name = "Maxon EPOS4/IDX"
    vendor_id = MAXON_VENDOR_ID

    # 16/32-bit entries only, even lengths, no dummy entries; the mode is set
    # over SDO. Target torque is per-mille of rated torque (standard CiA 402
    # unit), so CST is usable; target velocity is deliberately left out.
    def rx_pdo(self) -> PdoMap:
        if self.cfg.rx_pdo:
            return pdo_from_config(self.cfg.rx_pdo, self.cfg.rx_pdo_index)
        return PdoMap(self.cfg.rx_pdo_index, [
            PdoEntry("controlword", c.OD_CONTROLWORD, 0, 16),
            PdoEntry("target_position", c.OD_TARGET_POSITION, 0, 32, signed=True),
            PdoEntry("target_torque", c.OD_TARGET_TORQUE, 0, 16, signed=True),
        ])

    def tx_pdo(self) -> PdoMap:
        if self.cfg.tx_pdo:
            return pdo_from_config(self.cfg.tx_pdo, self.cfg.tx_pdo_index)
        return PdoMap(self.cfg.tx_pdo_index, [
            PdoEntry("statusword", c.OD_STATUSWORD, 0, 16),
            PdoEntry("position_actual", c.OD_POSITION_ACTUAL, 0, 32, signed=True),
            PdoEntry("torque_actual", c.OD_TORQUE_ACTUAL, 0, 16, signed=True),
        ])

    def configure_extra(self, slave: pysoem.CdefSlave) -> None:
        """Set the interpolation period and the drive-side error windows."""
        self.write_interpolation_period(slave)
        self._write_error_windows(slave)

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
        """Position limits (0x607D) only.

        Velocity objects (0x607F, 0x6080) are left at the drive's values: on
        the HEJ 70 they are in EPOS4 velocity units (0x60A9 = 0.001 rpm),
        apparently at the main-sensor (output) shaft. Writing 0x6080 as motor
        rpm (135) capped the joint at ~0.8 deg/s and it could not follow a
        30 deg move. The master's profile limits speed instead.
        """
        self.write_position_limits(slave)

    def read_gear_ratio(self, slave: pysoem.CdefSlave) -> float:
        """Gear reduction configured in the drive (0x3003), falling back to the
        axis gear_ratio when it cannot be read."""
        try:
            num = struct.unpack("<I", slave.sdo_read(OD_GEAR_CONFIG, 1)[:4])[0]
            den = struct.unpack("<I", slave.sdo_read(OD_GEAR_CONFIG, 2)[:4])[0]
            if num > 0 and den > 0:
                return num / den
        except Exception:  # noqa: BLE001
            log.debug("%s: 0x3003 not readable", self.cfg.name)
        return self.cfg.gear_ratio

    def read_rated_torque_mnm(self, slave: pysoem.CdefSlave) -> float:
        """Read 0x6076 (rated torque, mNm) so per-mille torque can be reported
        in physical units. Returns the configured value if the object is
        missing."""
        try:
            raw = slave.sdo_read(c.OD_MOTOR_RATED_TORQUE, 0)
            return float(struct.unpack("<I", raw[:4])[0])
        except Exception:  # noqa: BLE001
            return self.cfg.rated_torque_mnm
