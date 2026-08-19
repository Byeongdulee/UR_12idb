"""UR safety/robot mode enumerations.

These live in their own dependency-free module (only the stdlib ``enum``) so that
low-level drivers (urxe.robot, rtde.robot) can import ``SafetyMode`` without
importing robUR. robUR imports urxe at module load and urxe/__init__ eagerly
imports urxe.robot, so a ``from robUR import SafetyMode`` in the driver created a
cyclic/duplicate load of robUR. Depending on this leaf module instead breaks that
cycle. robUR re-exports these names for backward compatibility.
"""
from enum import Enum


class SafetyMode(Enum):
    IS_NORMAL_MODE = 1
    IS_REDUCED_MODE = 2
    IS_PROTECTIVE_STOPPED = 3
    IS_RECOVERY_MODE = 4
    IS_SAFEGUARD_STOPPED = 5
    IS_SYSTEM_EMERGENCY_STOPPED = 6
    IS_ROBOT_EMERGENCY_STOPPED = 7
    IS_EMERGENCY_STOPPED = 8
    IS_VIOLATION = 9
    IS_FAULT = 10
    IS_STOPPED_DUE_TO_SAFETY = 11


class SafetyStatus(Enum):
    SAFETY_STATUS_SYSTEM_THREE_POSITION_ENABLING_STOP = 13
    SAFETY_STATUS_AUTOMATIC_MODE_SAFEGUARD_STOP = 12
    SAFETY_STATUS_UNDEFINED_SAFETY_MODE = 11
    SAFETY_STATUS_VALIDATE_JOINT_ID = 10
    SAFETY_STATUS_FAULT = 9
    SAFETY_STATUS_VIOLATION = 8
    SAFETY_STATUS_ROBOT_EMERGENCY_STOP = 7
    SAFETY_STATUS_SYSTEM_EMERGENCY_STOP = 6
    SAFETY_STATUS_SAFEGUARD_STOP = 5
    SAFETY_STATUS_RECOVERY = 4
    SAFETY_STATUS_PROTECTIVE_STOP = 3
    SAFETY_STATUS_REDUCED = 2
    SAFETY_STATUS_NORMAL = 1


class RobotMode(Enum):
    ROBOT_MODE_NO_CONTROLLER = -1
    ROBOT_MODE_DISCONNECTED = 0
    ROBOT_MODE_CONFIRM_SAFETY = 1
    ROBOT_MODE_BOOTING = 2
    ROBOT_MODE_POWER_OFF = 3
    ROBOT_MODE_POWER_ON = 4
    ROBOT_MODE_IDLE = 5
    ROBOT_MODE_BACKDRIVE = 6
    ROBOT_MODE_RUNNING = 7
    ROBOT_MODE_UPDATING_FIRMWARE = 8
