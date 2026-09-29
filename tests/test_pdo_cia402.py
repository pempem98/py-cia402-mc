"""PDO packing and CiA 402 decoding. No hardware needed.

These two layers decide what bytes go on the wire and what the drive's reply
means, so the encodings are pinned against the standard here.
"""
import struct

import pytest

from ethercat_mc import cia402 as c
from ethercat_mc.pdo import (
    PdoEntry,
    PdoMap,
    default_rx_pdo,
    default_tx_pdo,
    minimal_rx_pdo,
    minimal_tx_pdo,
    pdo_from_config,
)


class TestPdoEntry:
    def test_mapping_value_layout(self):
        """index << 16 | subindex << 8 | bit length, per CiA 301."""
        e = PdoEntry("controlword", 0x6040, 0, 16)
        assert e.mapping_value == 0x60400010

    def test_mapping_value_with_subindex(self):
        e = PdoEntry("x", 0x60C2, 2, 8)
        assert e.mapping_value == 0x60C20208

    def test_target_position_is_32_bit_signed(self):
        e = PdoEntry("target_position", 0x607A, 0, 32, signed=True)
        assert e.mapping_value == 0x607A0020
        assert e.fmt == "i"
        assert e.nbytes == 4

    def test_unsupported_width_rejected(self):
        with pytest.raises(ValueError, match="unsupported width"):
            _ = PdoEntry("weird", 0x1234, 0, 24).fmt


class TestPdoMap:
    def test_size_matches_entries(self):
        pdo = minimal_rx_pdo()
        assert pdo.size == 6  # u16 + i32

    def test_pack_round_trip(self):
        pdo = minimal_rx_pdo()
        data = pdo.pack({"controlword": 0x000F, "target_position": -123456})
        assert data == struct.pack("<Hi", 0x000F, -123456)
        assert pdo.unpack(data) == {
            "controlword": 0x000F,
            "target_position": -123456,
        }

    def test_missing_values_pack_as_zero(self):
        pdo = minimal_rx_pdo()
        assert pdo.pack({"controlword": 6}) == struct.pack("<Hi", 6, 0)

    def test_negative_position_survives(self):
        """A signed target must not be mangled into a huge positive value."""
        pdo = minimal_rx_pdo()
        data = pdo.pack({"controlword": 0, "target_position": -1})
        assert pdo.unpack(data)["target_position"] == -1

    def test_unpack_ignores_trailing_padding(self):
        """Drives often report a larger image than we mapped."""
        pdo = minimal_tx_pdo()
        data = struct.pack("<Hi", 0x0637, 1000) + b"\x00" * 8
        assert pdo.unpack(data)["position_actual"] == 1000

    def test_unpack_rejects_short_image(self):
        pdo = minimal_tx_pdo()
        with pytest.raises(ValueError, match="expected >= 6 bytes"):
            pdo.unpack(b"\x00\x00")

    def test_default_rx_layout(self):
        """The full RxPDO carries every cyclic setpoint plus the mode."""
        pdo = default_rx_pdo()
        names = [e.name for e in pdo.entries]
        assert names == [
            "controlword", "target_position", "target_velocity",
            "target_torque", "mode_of_operation", "_pad",
        ]
        assert pdo.size == 2 + 4 + 4 + 2 + 1 + 1

    def test_default_tx_layout(self):
        pdo = default_tx_pdo()
        names = [e.name for e in pdo.entries]
        assert names == [
            "statusword", "position_actual", "velocity_actual",
            "torque_actual", "mode_display", "_pad",
        ]

    def test_default_layouts_have_even_length(self):
        """eRob rejects an odd sync manager length ("Invalid sync manager
        configuration") on the way to SAFE-OP, seen on real hardware."""
        assert default_rx_pdo().size % 2 == 0
        assert default_tx_pdo().size % 2 == 0

    def test_padding_entry_is_the_dummy_object(self):
        pad = default_rx_pdo().entries[-1]
        assert pad.mapping_value == 0x00000008

    def test_mode_of_operation_is_signed_byte(self):
        """0x6060 is INTEGER8: CSP is 8, but negative vendor modes exist."""
        pdo = default_rx_pdo()
        data = pdo.pack({"mode_of_operation": -1})
        assert pdo.unpack(data)["mode_of_operation"] == -1

    def test_index_of_and_has(self):
        pdo = default_rx_pdo()
        assert pdo.index_of("target_torque") == 3
        assert pdo.has("target_velocity")
        assert not pdo.has("nonexistent")
        with pytest.raises(KeyError):
            pdo.index_of("nonexistent")

    def test_pdo_from_config_accepts_hex_strings(self):
        pdo = pdo_from_config(
            [
                {"name": "controlword", "index": "0x6040", "bits": 16},
                {"name": "target_position", "index": "0x607A", "bits": 32,
                 "signed": True},
            ],
            0x1600,
        )
        assert pdo.entries[0].mapping_value == 0x60400010
        assert pdo.entries[1].mapping_value == 0x607A0020
        assert pdo.size == 6


class TestStateDecoding:
    @pytest.mark.parametrize("statusword,expected", [
        (0x0000, c.State.NOT_READY_TO_SWITCH_ON),
        (0x0040, c.State.SWITCH_ON_DISABLED),
        (0x0021, c.State.READY_TO_SWITCH_ON),
        (0x0023, c.State.SWITCHED_ON),
        (0x0027, c.State.OPERATION_ENABLED),
        (0x0007, c.State.QUICK_STOP_ACTIVE),
        (0x000F, c.State.FAULT_REACTION_ACTIVE),
        (0x0008, c.State.FAULT),
    ])
    def test_canonical_statuswords(self, statusword, expected):
        assert c.decode_state(statusword) is expected

    def test_decoding_ignores_unrelated_bits(self):
        """Voltage, remote and target-reached must not change the state."""
        base = 0x0027  # operation enabled
        for extra in (c.SW_VOLTAGE_ENABLED, c.SW_REMOTE, c.SW_TARGET_REACHED,
                      c.SW_WARNING, c.SW_INTERNAL_LIMIT):
            assert c.decode_state(base | extra) is c.State.OPERATION_ENABLED

    def test_real_drive_statusword(self):
        """0x0637: operation enabled, voltage on, remote, target reached."""
        assert c.decode_state(0x0637) is c.State.OPERATION_ENABLED

    def test_fault_takes_priority_over_ready_bits(self):
        """A fault must be reported even when the low bits look ready."""
        assert c.decode_state(0x0008) is c.State.FAULT
        assert c.decode_state(0x0028) is c.State.FAULT


class TestEnableSequence:
    def test_transitions_walk_to_operation_enabled(self):
        """From switch-on-disabled the sequence must be 0x06, 0x07, 0x0F."""
        assert c.next_controlword(c.State.SWITCH_ON_DISABLED) == 0x06
        assert c.next_controlword(c.State.READY_TO_SWITCH_ON) == 0x07
        assert c.next_controlword(c.State.SWITCHED_ON) == 0x0F

    def test_fault_is_reset_first(self):
        assert c.next_controlword(c.State.FAULT) == c.CW_FAULT_RESET_CMD

    def test_quick_stop_leaves_via_disable_voltage(self):
        assert c.next_controlword(c.State.QUICK_STOP_ACTIVE) == c.CW_DISABLE_VOLTAGE

    def test_no_transition_when_already_enabled(self):
        assert c.next_controlword(c.State.OPERATION_ENABLED) is None

    def test_full_sequence_converges(self):
        """Walking the machine from fault must reach operation enabled."""
        statuswords = {
            c.State.FAULT: 0x0040,               # reset -> switch on disabled
            c.State.SWITCH_ON_DISABLED: 0x0021,  # shutdown -> ready
            c.State.READY_TO_SWITCH_ON: 0x0023,  # switch on -> switched on
            c.State.SWITCHED_ON: 0x0027,         # enable -> operation enabled
        }
        state = c.State.FAULT
        for _ in range(10):
            cw = c.next_controlword(state)
            if cw is None:
                break
            state = c.decode_state(statuswords[state])
        assert state is c.State.OPERATION_ENABLED


class TestModes:
    def test_cyclic_modes_membership(self):
        assert c.Mode.CSP in c.CYCLIC_MODES
        assert c.Mode.CSV in c.CYCLIC_MODES
        assert c.Mode.CST in c.CYCLIC_MODES
        assert c.Mode.PP not in c.CYCLIC_MODES
        assert c.Mode.HOMING not in c.CYCLIC_MODES

    def test_mode_values_match_standard(self):
        assert int(c.Mode.PP) == 1
        assert int(c.Mode.HOMING) == 6
        assert int(c.Mode.CSP) == 8
        assert int(c.Mode.CSV) == 9
        assert int(c.Mode.CST) == 10

    def test_describe_statusword_mentions_state_and_flags(self):
        text = c.describe_statusword(0x0637)
        assert "Operation enabled" in text
        assert "target-reached" in text
        assert "0x0637" in text
