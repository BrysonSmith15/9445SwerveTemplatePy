from swerve_module import SwerveModule

from commands2 import Subsystem

from wpimath.geometry import Translation2d
from wpimath.units import inchesToMeters
from wpimath.kinematics import SwerveDrive4Kinematics


class Drivetrain(Subsystem):
    def __init__(self):
        """kinematics"""
        # module locations
        # TODO: make the signs correct for backs. Not sure which way needs to be forwards
        fl_position = Translation2d(inchesToMeters(12), inchesToMeters(12))
        fr_position = Translation2d(inchesToMeters(12), -inchesToMeters(12))
        bl_position = Translation2d(-inchesToMeters(12), inchesToMeters(12))
        br_position = Translation2d(-inchesToMeters(12), -inchesToMeters(12))

        self.kinematics = SwerveDrive4Kinematics(
            fl_position, fr_position, bl_position, br_position)
