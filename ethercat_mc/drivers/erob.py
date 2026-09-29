"""ZeroErr eRob integrated joint actuators.

eRob exposes a 19-bit absolute encoder on the *output* shaft, so positions are
already in output coordinates and `gear_ratio` stays 1.0 (the harmonic drive is
behind the encoder). Verify counts_per_rev against the manual for your model:
eRob70/80/110 are commonly 524288 (2^19), but some variants differ.
"""
from __future__ import annotations

import logging
import struct

import pysoem

from .. import cia402 as c
from .base import Driver, register

log = logging.getLogger(__name__)

#: ZeroErr vendor ID as reported on the bus.
ZEROERR_VENDOR_ID = 0x5A65726F

# --- eRob-specific objects ----------------------------------------------
#: Interpolation time period (0x60C2:1 value, :2 exponent). CSP needs this to
#: match the master's cycle time or the drive's interpolator drifts.
OD_INTERPOLATION_TIME = 0x60C2


@register
class ERobDriver(Driver):
    key = "erob"
    display_name = "ZeroErr eRob"
    vendor_id = None  # eRob firmware revisions report different IDs; see below

    def configure_extra(self, slave: pysoem.CdefSlave) -> None:
        """Set the interpolation time period to the master's cycle time.

        In CSP the drive interpolates between the positions the master sends.
        If 0x60C2 does not match the real cycle time, the drive either lags or
        overshoots between frames, which shows up as vibration at the cycle
        frequency.
        """
        self._write_interpolation_period(slave)

    def _write_interpolation_period(self, slave: pysoem.CdefSlave) -> None:
        # The bus config is not visible from the driver, so the cycle time
        # comes through startup_sdo when it must differ from the 2 ms default.
        already_set = any(
            (int(i["index"], 0) if isinstance(i["index"], str) else i["index"])
            == OD_INTERPOLATION_TIME
            for i in self.cfg.startup_sdo
        )
        if already_set:
            return
        try:
            # 2 ms = 2 * 10^-3 s
            slave.sdo_write(OD_INTERPOLATION_TIME, 1, struct.pack("<B", 2))
            slave.sdo_write(OD_INTERPOLATION_TIME, 2, struct.pack("<b", -3))
            log.debug("%s: interpolation time period set to 2 ms", self.cfg.name)
        except Exception:  # noqa: BLE001 - not present on all firmware
            log.debug("%s: 0x60C2 not writable, leaving default", self.cfg.name)

    def check_identity(self, slave: pysoem.CdefSlave) -> None:
        """eRob firmware revisions report different vendor IDs, so only an
        explicit `vendor_id` in the configuration is enforced. The name is
        logged so a mismatch is still visible."""
        super().check_identity(slave)
        if self.cfg.vendor_id is None:
            log.info(
                "%s: eRob at slave %d reports vendor 0x%08X product 0x%08X (%s); "
                "pin these in the config once confirmed",
                self.cfg.name, self.cfg.slave_position, slave.man, slave.id,
                slave.name,
            )
