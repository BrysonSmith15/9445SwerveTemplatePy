from wpimath.units import (
    meters,
    inchesToMeters,
    meters_per_second,
    feetToMeters,
    radians_per_second,
    degreesToRadians,
    kilograms,
    lbsToKilograms,
)

from ntcore.util import ntproperty


class ModuleConstants:
    drive_id: int
    turn_id: int
    cancoder_id: int

    def __init__(self, drive: int, turn: int, cancoder: int):
        self.drive_id = drive
        self.turn_id = turn
        self.cancoder_id = cancoder


class SwerveConstants:
    front_left: ModuleConstants = ModuleConstants(1, 2, 3)
    front_right: ModuleConstants = ModuleConstants(1, 2, 3)
    back_left: ModuleConstants = ModuleConstants(1, 2, 3)
    back_right: ModuleConstants = ModuleConstants(1, 2, 3)

    drive_ratio: float = 8.14
    turn_ratio: float = 150 / 7

    drivebase_width: meters = inchesToMeters(28)
    drivebase_length: meters = inchesToMeters(28)
    mass: kilograms = lbsToKilograms(120)

    max_speed = ntproperty("max_speed", feetToMeters(15))
    max_angular_speed = ntproperty("max_angular_speed", degreesToRadians(270))
