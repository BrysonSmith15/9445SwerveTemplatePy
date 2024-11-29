from swerve_module import SwerveModule

from commands2 import Subsystem

from wpimath.geometry import Translation2d
from wpimath.units import inchesToMeters
from wpimath.kinematics import SwerveDrive4Kinematics


class Drivetrain(Subsystem):
    def __init__(self):
        """member instantiation"""
        # TODO: Update these values
        self.fl = SwerveModule("fl", 3, 4, 5)
        self.fr = SwerveModule("fl", 4, 5, 6)
        self.bl = SwerveModule("fl", 7, 8, 9)
        self.br = SwerveModule("fl", 10, 11, 12)
        """kinematics"""
        # module locations
        # TODO: make the signs correct for backs. Not sure which way needs to be forwards
        fl_position = Translation2d(inchesToMeters(12), inchesToMeters(12))
        fr_position = Translation2d(inchesToMeters(12), -inchesToMeters(12))
        bl_position = Translation2d(-inchesToMeters(12), inchesToMeters(12))
        br_position = Translation2d(-inchesToMeters(12), -inchesToMeters(12))

        self.kinematics = SwerveDrive4Kinematics(
            fl_position, fr_position, bl_position, br_position)

        """odometry"""
