from swerve_module import SwerveModule
from gyro_base import GyroBase
from navx_gryo import NavX

from commands2 import Subsystem

from wpimath.geometry import Translation2d
from wpimath.units import inchesToMeters
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry


class Drivetrain(Subsystem):
    def __init__(self):
        """member instantiation"""
        # TODO: Update these values
        self.fl = SwerveModule("fl", 3, 4, 5, False, False)
        self.fr = SwerveModule("fl", 4, 5, 6, False, False)
        self.bl = SwerveModule("fl", 7, 8, 9, False, False)
        self.br = SwerveModule("fl", 10, 11, 12, False, False)

        self.gryo = NavX.fromMXP()

        """kinematics"""
        # module locations
        fl_position = Translation2d(inchesToMeters(12), inchesToMeters(12))
        fr_position = Translation2d(inchesToMeters(12), -inchesToMeters(12))
        bl_position = Translation2d(-inchesToMeters(12), inchesToMeters(12))
        br_position = Translation2d(-inchesToMeters(12), -inchesToMeters(12))

        # kinematics - turns robot speeds into individual states for each module
        self.kinematics = SwerveDrive4Kinematics(
            fl_position, fr_position, bl_position, br_position
        )

        """odometry"""
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.get_angle(),
            (
                self.fl.get_position(),
                self.fr.get_position(),
                self.bl.get_position(),
                self.br.get_position(),
            ),
        )
