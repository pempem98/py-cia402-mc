"""Driver base class: the vendor-specific part of bringing a slave up.

A driver supplies the PDO layout and the PRE-OP configuration for one family
of drives. Everything above it — the CiA 402 state machine, setpoint
generation, safety — is vendor neutral and lives in `Axis`.
"""
from __future__ import annotations

import logging
import struct

import pysoem

from .. import cia402 as c
from ..config import AxisConfig
from ..pdo import (
    PdoMap,
    default_rx_pdo,
    default_tx_pdo,
    pdo_from_config,
)

log = logging.getLogger(__name__)

#: struct formats for the `type` field of a startup_sdo entry in YAML.
SDO_TYPES = {
    "u8": "<B", "i8": "<b",
    "u16": "<H", "i16": "<h",
    "u32": "<I", "i32": "<i",
    "u64": "<Q", "i64": "<q",
}


class Driver:
    """Base driver. Subclasses override the class attributes and hooks."""

    #: Key used in the YAML `driver:` field.
    key = "generic"
    #: Human-readable name, for logs.
    display_name = "Generic CiA 402"
    #: Expected vendor ID, or None to skip the check.
    vendor_id: int | None = None

    def __init__(self, cfg: AxisConfig, cycle_time_s: float = 0.002):
        self.cfg = cfg
        #: Bus cycle time; drives that interpolate in CSP need to know it.
        self.cycle_time_s = cycle_time_s

    def write_interpolation_period(self, slave: pysoem.CdefSlave) -> None:
        """Set 0x60C2 (interpolation time period) to the bus cycle time.

        In CSP the drive interpolates between setpoints over this period; if
        it differs from the real cycle the motion stutters, and EPOS4 can
        report EtherCAT errors. A startup_sdo entry for 0x60C2 takes priority.
        """
        if any((int(i["index"], 0) if isinstance(i["index"], str) else i["index"])
               == 0x60C2 for i in self.cfg.startup_sdo):
            return
        ms = int(round(self.cycle_time_s * 1000))
        try:
            slave.sdo_write(0x60C2, 1, struct.pack("<B", ms))
            slave.sdo_write(0x60C2, 2, struct.pack("<b", -3))
            log.debug("%s: interpolation period %d ms", self.cfg.name, ms)
        except Exception:  # noqa: BLE001 - optional on some firmware
            log.debug("%s: 0x60C2 not writable", self.cfg.name)

    # --- PDO layout ------------------------------------------------------
    def rx_pdo(self) -> PdoMap:
        """Master -> drive mapping. Config overrides the driver default."""
        if self.cfg.rx_pdo:
            return pdo_from_config(self.cfg.rx_pdo, self.cfg.rx_pdo_index)
        return default_rx_pdo(self.cfg.rx_pdo_index)

    def tx_pdo(self) -> PdoMap:
        """Drive -> master mapping. Config overrides the driver default."""
        if self.cfg.tx_pdo:
            return pdo_from_config(self.cfg.tx_pdo, self.cfg.tx_pdo_index)
        return default_tx_pdo(self.cfg.tx_pdo_index)

    # --- identity --------------------------------------------------------
    def check_identity(self, slave: pysoem.CdefSlave) -> None:
        """Fail early if the slave at this position is not what the config says.

        Wiring mistakes are the most common cause of a machine moving the wrong
        joint, so this is checked before anything is enabled.
        """
        expected_vendor = self.cfg.vendor_id or self.vendor_id
        if expected_vendor is not None and slave.man != expected_vendor:
            raise RuntimeError(
                f"axis {self.cfg.name} at slave {self.cfg.slave_position}: "
                f"expected vendor 0x{expected_vendor:08X}, found 0x{slave.man:08X} "
                f"({slave.name}) - check the wiring order"
            )
        if self.cfg.product_code is not None and slave.id != self.cfg.product_code:
            raise RuntimeError(
                f"axis {self.cfg.name} at slave {self.cfg.slave_position}: "
                f"expected product 0x{self.cfg.product_code:08X}, "
                f"found 0x{slave.id:08X} ({slave.name})"
            )

    # --- PRE-OP configuration --------------------------------------------
    def make_config_func(self, slave: pysoem.CdefSlave):
        """Return the callable SOEM invokes while this slave is in PRE-OP.

        SOEM passes the slave position; the slave object is captured here so
        subclasses do not have to look it up again.
        """
        def configure(_position: int) -> None:
            self.configure(slave)

        return configure

    def configure(self, slave: pysoem.CdefSlave) -> None:
        """Write the PDO mapping and any startup objects. PRE-OP only."""
        self.write_pdo_mapping(slave, self.rx_pdo(), c.OD_SM2_ASSIGN)
        self.write_pdo_mapping(slave, self.tx_pdo(), c.OD_SM3_ASSIGN)
        self.write_startup_sdos(slave)
        self.configure_extra(slave)
        log.info("%s: configured as %s", self.cfg.name, self.display_name)

    def configure_extra(self, slave: pysoem.CdefSlave) -> None:
        """Hook for vendor-specific objects. Default does nothing."""

    @staticmethod
    def write_pdo_mapping(
        slave: pysoem.CdefSlave, pdo: PdoMap, assign_index: int
    ) -> None:
        """Write one PDO mapping and assign it to its sync manager.

        The order matters and is fixed by the standard: clear the assignment,
        clear the mapping, write the entries, set the entry count, assign the
        PDO, then set the assignment count. Writing a mapping while its count
        is non-zero is rejected by most drives.
        """
        # Clear the sync manager assignment.
        slave.sdo_write(assign_index, 0, struct.pack("<B", 0))
        # Clear the mapping object, then fill it.
        slave.sdo_write(pdo.mapping_index, 0, struct.pack("<B", 0))
        for i, entry in enumerate(pdo.entries, start=1):
            slave.sdo_write(
                pdo.mapping_index, i, struct.pack("<I", entry.mapping_value)
            )
        slave.sdo_write(
            pdo.mapping_index, 0, struct.pack("<B", len(pdo.entries))
        )
        # Assign the PDO to the sync manager.
        slave.sdo_write(assign_index, 1, struct.pack("<H", pdo.mapping_index))
        slave.sdo_write(assign_index, 0, struct.pack("<B", 1))

    def write_startup_sdos(self, slave: pysoem.CdefSlave) -> None:
        """Apply the `startup_sdo` list from the axis configuration."""
        for item in self.cfg.startup_sdo:
            index = item["index"]
            index = int(index, 0) if isinstance(index, str) else index
            subindex = item.get("subindex", 0)
            type_name = item.get("type", "u32")
            if type_name not in SDO_TYPES:
                raise ValueError(
                    f"axis {self.cfg.name}: startup_sdo type {type_name!r} is not one "
                    f"of {', '.join(SDO_TYPES)}"
                )
            value = item["value"]
            value = int(value, 0) if isinstance(value, str) else value
            slave.sdo_write(
                index, subindex, struct.pack(SDO_TYPES[type_name], value)
            )
            log.debug("%s: startup SDO 0x%04X:%d = %s",
                      self.cfg.name, index, subindex, value)

    # --- runtime parameters (SAFE-OP / OP, over SDO) ----------------------
    def apply_motion_limits(self, slave: pysoem.CdefSlave) -> None:
        """Push the configured limits into the drive's own objects.

        These are a second line of defence: the master already limits motion,
        but a drive that enforces its own limits also protects against a master
        that stops sending or sends nonsense.
        """
        self.write_max_profile_velocity(slave)
        self.write_position_limits(slave)

    def write_max_profile_velocity(self, slave: pysoem.CdefSlave) -> None:
        """0x607F in counts/s. Only valid for drives whose velocity unit is
        position-units per second; drivers with another unit override this."""
        limits = self.cfg.limits
        max_velocity = self.cfg.velocity_to_counts(limits.max_velocity_deg_s)
        try:
            slave.sdo_write(
                c.OD_MAX_PROFILE_VELOCITY, 0, struct.pack("<I", max_velocity)
            )
        except Exception:  # noqa: BLE001 - optional object on some drives
            log.debug("%s: 0x607F not writable", self.cfg.name)

    def write_position_limits(self, slave: pysoem.CdefSlave) -> None:
        """0x607D software position limits, in raw counts around the zero."""
        limits = self.cfg.limits
        if limits.min_deg is not None and limits.max_deg is not None:
            lo = self.cfg.deg_to_counts(limits.min_deg)
            hi = self.cfg.deg_to_counts(limits.max_deg)
            if lo > hi:  # a negative `direction` flips the order
                lo, hi = hi, lo
            # Never let the drive see a window that excludes the current
            # position, even between the two writes: EPOS4 faults at once
            # (0x8A82). So skip a window that would not contain it, and open
            # the window fully before narrowing it to the new bounds.
            try:
                pos = struct.unpack(
                    "<i", slave.sdo_read(c.OD_POSITION_ACTUAL, 0)[:4])[0]
            except Exception:  # noqa: BLE001
                pos = None
            if pos is not None and not lo <= pos <= hi:
                log.warning(
                    "%s: not writing drive limits [%d, %d]: position %d is "
                    "outside them (zero the axis first); master-side limits "
                    "still apply", self.cfg.name, lo, hi, pos)
                return
            try:
                slave.sdo_write(c.OD_SW_POS_LIMIT, 1, struct.pack("<i", -2**31))
                slave.sdo_write(c.OD_SW_POS_LIMIT, 2, struct.pack("<i", 2**31 - 1))
                slave.sdo_write(c.OD_SW_POS_LIMIT, 1, struct.pack("<i", lo))
                slave.sdo_write(c.OD_SW_POS_LIMIT, 2, struct.pack("<i", hi))
                log.info("%s: drive position limits set to [%d, %d] counts",
                         self.cfg.name, lo, hi)
            except Exception:  # noqa: BLE001 - optional object
                log.debug("%s: 0x607D not writable", self.cfg.name)

    def read_supported_modes(self, slave: pysoem.CdefSlave) -> set[c.Mode]:
        """Decode object 0x6502 into the set of modes the drive supports."""
        try:
            raw = slave.sdo_read(c.OD_SUPPORTED_DRIVE_MODES, 0)
        except Exception:  # noqa: BLE001 - optional object
            return set()
        bits = struct.unpack("<I", raw[:4])[0]
        # 0x6502 bit n set means mode n+1 is supported.
        supported = set()
        for mode in c.Mode:
            if mode is c.Mode.NO_MODE:
                continue
            if bits & (1 << (int(mode) - 1)):
                supported.add(mode)
        return supported


_REGISTRY: dict[str, type[Driver]] = {}


def register(cls: type[Driver]) -> type[Driver]:
    """Class decorator that adds a driver to the registry."""
    _REGISTRY[cls.key] = cls
    return cls


def get_driver(cfg: AxisConfig, cycle_time_s: float = 0.002) -> Driver:
    """Instantiate the driver named by `cfg.driver`."""
    try:
        cls = _REGISTRY[cfg.driver]
    except KeyError:
        raise ValueError(
            f"axis {cfg.name}: unknown driver {cfg.driver!r}; available: "
            f"{', '.join(sorted(_REGISTRY))}"
        ) from None
    return cls(cfg, cycle_time_s)


def available_drivers() -> list[str]:
    return sorted(_REGISTRY)


register(Driver)
