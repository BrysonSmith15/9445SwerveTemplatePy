from subsystems.swerve_module import SwerveModule

from commands2 import Command

from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d


class RobotContainer:
    def __init__(self) -> None:
        my_module = SwerveModule("fl", 5, 6, 7, False, False)

    def get_auto_command(self) -> Command:
        return Command()
