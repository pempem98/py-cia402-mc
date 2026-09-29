"""CiA 402 (DS402) definitions: object dictionary indices, state machine, modes.

This layer is vendor neutral. Both eRob and Maxon implement CiA 402, so the
state machine and the object indices below apply to every drive on the bus.
Vendor differences live in `drivers/`.
"""
from __future__ import annotations

import enum

# --- Object dictionary: standard CiA 402 indices -------------------------
OD_ERROR_CODE = 0x603F
OD_CONTROLWORD = 0x6040
OD_STATUSWORD = 0x6041
OD_MODE_OF_OP = 0x6060
OD_MODE_OF_OP_DISPLAY = 0x6061
OD_POSITION_ACTUAL = 0x6064
OD_VELOCITY_ACTUAL = 0x606C
OD_TARGET_TORQUE = 0x6071
OD_MOTOR_RATED_TORQUE = 0x6076
OD_TORQUE_ACTUAL = 0x6077
OD_TARGET_POSITION = 0x607A
OD_SW_POS_LIMIT = 0x607D
OD_MAX_PROFILE_VELOCITY = 0x607F
OD_PROFILE_VELOCITY = 0x6081
OD_PROFILE_ACCELERATION = 0x6083
OD_PROFILE_DECELERATION = 0x6084
OD_QUICKSTOP_DECELERATION = 0x6085
OD_HOMING_METHOD = 0x6098
OD_HOMING_SPEEDS = 0x6099
OD_HOMING_ACCELERATION = 0x609A
OD_TARGET_VELOCITY = 0x60FF
OD_SUPPORTED_DRIVE_MODES = 0x6502

# --- SyncManager / PDO assignment objects --------------------------------
OD_SM2_ASSIGN = 0x1C12  # RxPDO assign (master -> slave)
OD_SM3_ASSIGN = 0x1C13  # TxPDO assign (slave -> master)


class Mode(enum.IntEnum):
    """Modes of operation (object 0x6060)."""

    NO_MODE = 0
    PP = 1  # Profile Position
    PV = 3  # Profile Velocity
    PT = 4  # Profile Torque
    HOMING = 6
    IP = 7  # Interpolated Position
    CSP = 8  # Cyclic Synchronous Position
    CSV = 9  # Cyclic Synchronous Velocity
    CST = 10  # Cyclic Synchronous Torque


#: Modes where the master must supply a fresh setpoint every cycle.
CYCLIC_MODES = frozenset({Mode.CSP, Mode.CSV, Mode.CST, Mode.IP})


class State(enum.Enum):
    """CiA 402 drive states, decoded from the statusword."""

    NOT_READY_TO_SWITCH_ON = "Not ready to switch on"
    SWITCH_ON_DISABLED = "Switch on disabled"
    READY_TO_SWITCH_ON = "Ready to switch on"
    SWITCHED_ON = "Switched on"
    OPERATION_ENABLED = "Operation enabled"
    QUICK_STOP_ACTIVE = "Quick stop active"
    FAULT_REACTION_ACTIVE = "Fault reaction active"
    FAULT = "Fault"
    UNKNOWN = "Unknown"


# Statusword bits
SW_READY_TO_SWITCH_ON = 1 << 0
SW_SWITCHED_ON = 1 << 1
SW_OPERATION_ENABLED = 1 << 2
SW_FAULT = 1 << 3
SW_VOLTAGE_ENABLED = 1 << 4
SW_QUICK_STOP = 1 << 5
SW_SWITCH_ON_DISABLED = 1 << 6
SW_WARNING = 1 << 7
SW_REMOTE = 1 << 9
SW_TARGET_REACHED = 1 << 10
SW_INTERNAL_LIMIT = 1 << 11
#: Mode-specific bit 12: "set-point acknowledge" (PP), "homing attained" (homing).
SW_SETPOINT_ACK = 1 << 12
SW_HOMING_ATTAINED = 1 << 12
#: Mode-specific bit 13: "following error" (PP/CSP), "homing error" (homing).
SW_FOLLOWING_ERROR = 1 << 13
SW_HOMING_ERROR = 1 << 13

# Controlword bits
CW_SWITCH_ON = 1 << 0
CW_ENABLE_VOLTAGE = 1 << 1
CW_QUICK_STOP = 1 << 2  # active low: 0 triggers quick stop
CW_ENABLE_OPERATION = 1 << 3
CW_NEW_SETPOINT = 1 << 4  # PP; also "homing start" in homing mode
CW_HOMING_START = 1 << 4
CW_CHANGE_SET_IMMEDIATELY = 1 << 5
CW_RELATIVE = 1 << 6
CW_FAULT_RESET = 1 << 7
CW_HALT = 1 << 8

# Common controlword commands
CW_SHUTDOWN = 0x06  # -> Ready to switch on
CW_SWITCH_ON_CMD = 0x07  # -> Switched on
CW_ENABLE_OPERATION_CMD = 0x0F  # -> Operation enabled
CW_DISABLE_VOLTAGE = 0x00  # -> Switch on disabled
CW_QUICK_STOP_CMD = 0x02  # -> Quick stop active
CW_FAULT_RESET_CMD = 0x80


def decode_state(statusword: int) -> State:
    """Decode a statusword into a CiA 402 state.

    The state is identified by bits 0-3, 5 and 6. Bit 6 (switch on disabled)
    and the fault bits take priority, so the masks must be tested in order.
    """
    if statusword & 0x4F == 0x00:
        return State.NOT_READY_TO_SWITCH_ON
    if statusword & 0x4F == 0x40:
        return State.SWITCH_ON_DISABLED
    if statusword & 0x6F == 0x21:
        return State.READY_TO_SWITCH_ON
    if statusword & 0x6F == 0x23:
        return State.SWITCHED_ON
    if statusword & 0x6F == 0x27:
        return State.OPERATION_ENABLED
    if statusword & 0x6F == 0x07:
        return State.QUICK_STOP_ACTIVE
    if statusword & 0x4F == 0x0F:
        return State.FAULT_REACTION_ACTIVE
    if statusword & 0x4F == 0x08:
        return State.FAULT
    return State.UNKNOWN


def next_controlword(current: State) -> int | None:
    """Return the controlword that moves `current` one step toward
    OPERATION_ENABLED, or None if no transition is needed or possible.
    """
    if current is State.FAULT:
        return CW_FAULT_RESET_CMD
    if current is State.SWITCH_ON_DISABLED:
        return CW_SHUTDOWN
    if current is State.READY_TO_SWITCH_ON:
        return CW_SWITCH_ON_CMD
    if current is State.SWITCHED_ON:
        return CW_ENABLE_OPERATION_CMD
    if current is State.QUICK_STOP_ACTIVE:
        # Leave quick stop via disable voltage, then re-run the sequence.
        return CW_DISABLE_VOLTAGE
    return None


def describe_statusword(statusword: int) -> str:
    """Human-readable summary of a statusword, for logs and the CLI."""
    flags = []
    if statusword & SW_VOLTAGE_ENABLED:
        flags.append("voltage")
    if statusword & SW_WARNING:
        flags.append("warning")
    if statusword & SW_REMOTE:
        flags.append("remote")
    if statusword & SW_TARGET_REACHED:
        flags.append("target-reached")
    if statusword & SW_INTERNAL_LIMIT:
        flags.append("internal-limit")
    state = decode_state(statusword).value
    return f"0x{statusword:04X} {state}" + (f" [{', '.join(flags)}]" if flags else "")
