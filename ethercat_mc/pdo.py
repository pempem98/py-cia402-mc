"""PDO mapping: describe entries, build the SDO writes that configure them,
and pack/unpack the process-data image for one slave.

A PDO entry is written to the mapping object as a 32-bit value:
    bits 31..16 = object index, bits 15..8 = subindex, bits 7..0 = bit length.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

#: Bit length -> struct format character, for signed and unsigned entries.
_FMT = {
    (8, False): "B", (8, True): "b",
    (16, False): "H", (16, True): "h",
    (32, False): "I", (32, True): "i",
    (64, False): "Q", (64, True): "q",
}


@dataclass(frozen=True)
class PdoEntry:
    """One mapped object inside a PDO."""

    name: str  #: key used to address this entry from application code
    index: int
    subindex: int = 0
    bits: int = 32
    signed: bool = False

    @property
    def mapping_value(self) -> int:
        """The 32-bit value written into the PDO mapping object."""
        return (self.index << 16) | (self.subindex << 8) | self.bits

    @property
    def fmt(self) -> str:
        """struct format character for this entry."""
        try:
            return _FMT[(self.bits, self.signed)]
        except KeyError:
            raise ValueError(
                f"PDO entry {self.name}: unsupported width {self.bits} bits"
            ) from None

    @property
    def nbytes(self) -> int:
        return self.bits // 8


@dataclass
class PdoMap:
    """One PDO (an ordered list of entries) bound to a mapping object."""

    #: Mapping object index, e.g. 0x1600 for RxPDO1, 0x1A00 for TxPDO1.
    mapping_index: int
    entries: list[PdoEntry] = field(default_factory=list)

    @property
    def struct_fmt(self) -> str:
        return "<" + "".join(e.fmt for e in self.entries)

    @property
    def size(self) -> int:
        return struct.calcsize(self.struct_fmt)

    def index_of(self, name: str) -> int:
        for i, e in enumerate(self.entries):
            if e.name == name:
                return i
        raise KeyError(f"no PDO entry named {name!r} in 0x{self.mapping_index:04X}")

    def has(self, name: str) -> bool:
        return any(e.name == name for e in self.entries)

    def pack(self, values: dict[str, int]) -> bytes:
        """Pack a name->value dict into the process-data bytes for this PDO.

        Entries missing from `values` are sent as 0; that is the correct
        default for a controlword or a setpoint that has not been set yet.
        """
        return struct.pack(
            self.struct_fmt, *(int(values.get(e.name, 0)) for e in self.entries)
        )

    def unpack(self, data: bytes) -> dict[str, int]:
        """Unpack process-data bytes into a name->value dict.

        Slaves may report more bytes than we mapped (padding), so only the
        leading `size` bytes are decoded.
        """
        size = self.size
        if len(data) < size:
            raise ValueError(
                f"PDO 0x{self.mapping_index:04X}: expected >= {size} bytes, got {len(data)}"
            )
        values = struct.unpack(self.struct_fmt, data[:size])
        return {e.name: v for e, v in zip(self.entries, values)}


def pad_to_even(pdo: PdoMap) -> PdoMap:
    """Append an 8-bit gap entry if the PDO has an odd byte length.

    eRob (and many other drives) reject an odd sync manager length with AL
    status "Invalid sync manager configuration" on the way to SAFE-OP. Object
    0x0000 is the CiA 301 dummy entry: it reserves space and carries nothing.
    """
    if pdo.size % 2:
        pdo.entries.append(PdoEntry("_pad", 0x0000, 0, 8))
    return pdo


# --- Standard PDO layouts ------------------------------------------------
# Names here are the contract between drivers and the motion layer: the axis
# code looks up "controlword", "target_position", ... regardless of vendor.

def default_rx_pdo(mapping_index: int = 0x1600) -> PdoMap:
    """Master -> drive: controlword plus setpoints for CSP, CSV and CST.

    All three setpoints are mapped so the mode can be switched at runtime
    without re-doing the PDO configuration (which requires dropping to PRE-OP).
    Unused setpoints are simply sent as 0 and ignored by the drive.
    """
    from . import cia402 as c

    return pad_to_even(PdoMap(
        mapping_index,
        [
            PdoEntry("controlword", c.OD_CONTROLWORD, 0, 16, signed=False),
            PdoEntry("target_position", c.OD_TARGET_POSITION, 0, 32, signed=True),
            PdoEntry("target_velocity", c.OD_TARGET_VELOCITY, 0, 32, signed=True),
            PdoEntry("target_torque", c.OD_TARGET_TORQUE, 0, 16, signed=True),
            PdoEntry("mode_of_operation", c.OD_MODE_OF_OP, 0, 8, signed=True),
        ],
    ))


def default_tx_pdo(mapping_index: int = 0x1A00) -> PdoMap:
    """Drive -> master: statusword, actual values and the active mode."""
    from . import cia402 as c

    return pad_to_even(PdoMap(
        mapping_index,
        [
            PdoEntry("statusword", c.OD_STATUSWORD, 0, 16, signed=False),
            PdoEntry("position_actual", c.OD_POSITION_ACTUAL, 0, 32, signed=True),
            PdoEntry("velocity_actual", c.OD_VELOCITY_ACTUAL, 0, 32, signed=True),
            PdoEntry("torque_actual", c.OD_TORQUE_ACTUAL, 0, 16, signed=True),
            PdoEntry("mode_display", c.OD_MODE_OF_OP_DISPLAY, 0, 8, signed=True),
        ],
    ))


def minimal_rx_pdo(mapping_index: int = 0x1600) -> PdoMap:
    """Controlword + target position only, for drives that reject larger maps."""
    from . import cia402 as c

    return PdoMap(
        mapping_index,
        [
            PdoEntry("controlword", c.OD_CONTROLWORD, 0, 16, signed=False),
            PdoEntry("target_position", c.OD_TARGET_POSITION, 0, 32, signed=True),
        ],
    )


def minimal_tx_pdo(mapping_index: int = 0x1A00) -> PdoMap:
    """Statusword + actual position only."""
    from . import cia402 as c

    return PdoMap(
        mapping_index,
        [
            PdoEntry("statusword", c.OD_STATUSWORD, 0, 16, signed=False),
            PdoEntry("position_actual", c.OD_POSITION_ACTUAL, 0, 32, signed=True),
        ],
    )


def pdo_from_config(spec: list[dict], mapping_index: int) -> PdoMap:
    """Build a PdoMap from the entry list in a YAML axis definition."""
    entries = [
        PdoEntry(
            name=e["name"],
            index=int(e["index"], 0) if isinstance(e["index"], str) else e["index"],
            subindex=e.get("subindex", 0),
            bits=e.get("bits", 32),
            signed=e.get("signed", False),
        )
        for e in spec
    ]
    return PdoMap(mapping_index, entries)
