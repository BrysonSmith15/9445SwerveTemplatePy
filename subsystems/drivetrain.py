from subsystems.swerve_module import SwerveModule
from subsystems.navx_gryo import NavX

from commands2 import Subsystem, InstantCommand, StartEndCommand, InterruptionBehavior, RunCommand

from wpilib import Field2d
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.units import inchesToMeters, feetToMeters, metersToFeet
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition, ChassisSpeeds, SwerveModuleState
from wpimath import applyDeadband

from ntcore import NetworkTableInstance, EventFlags

import typing


class Drivetrain(Subsystem):
    def __init__(self):
        """member instantiation"""
        # TODO: Update these values
        self.fl = SwerveModule("fl", 14,12 , 13, False, False)
        self.fr = SwerveModule("fr", 11, 9, 10, False, False)
        self.bl = SwerveModule("bl", 17, 15, 7, False, False)
        self.br = SwerveModule("br", 8, 6, 16, False, False)

        self.gyro = NavX.fromMXP()

        self.max_velocity_mps = feetToMeters(4)
        self.max_angular_velocity = Rotation2d.fromDegrees(90)

        """nettables"""
        self.nettable = NetworkTableInstance.getDefault().getTable("Drivetrain")
        self.nettable.putNumber(
            "config/max_velocity_fps", metersToFeet(self.max_velocity_mps))
        self.nettable.putNumber(
            "config/max_angular_velocity_degps", self.max_angular_velocity.degrees())
        # TODO: setattr is bad. Fix it probably with a full function. Could be nested in __init__
        self.nettable.addListener("max_velocity_fps", EventFlags.kValueAll, lambda _, __, ev: setattr(
            self, "max_velocity_mps", feetToMeters(ev.data.value.value())))
        self.nettable.addListener("max_angular_velocity_degps", EventFlags.kValueAll, lambda _, __, ev: setattr(
            self, "max_angular_velocity", Rotation2d.fromDegrees(ev.data.value.value())))

        self.field = Field2d()
        # these should be changed when actually using the field
        self.field.setRobotPose(Pose2d(10, 10, 0))
        # TODO: x, y, theta velocity PIDs and respective event listeners on nettable

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

    def periodic(self) -> None:
        self.odometry.update(
            self.gyro.get_angle(),
            (
                self.fl.get_position(),
                self.fr.get_position(),
                self.bl.get_position(),
                self.br.get_position()
            )
        )

        curr_speed = self.kinematics.toChassisSpeeds(
            (
                self.fl.get_state(),
                self.fr.get_state(),
                self.bl.get_state(),
                self.br.get_state()
            )
        )
        self.nettable.putNumber("velocity/vx (fps)", curr_speed.vx_fps)
        self.nettable.putNumber("velocity/vy (fps)", curr_speed.vy_fps)
        self.nettable.putNumber("velocity/omega (degps)", curr_speed.omega_dps)
        if self.getCurrentCommand():
            self.nettable.putString("Running Command", self.getCurrentCommand().getName())
        else:
            self.nettable.putString("Running Command", "None")

    """getters"""

    def get_kinematics(self) -> SwerveDrive4Kinematics:
        return self.kinematics

    def get_odometry(self) -> SwerveDrive4Odometry:
        return self.odometry

    def get_pose(self) -> Pose2d:
        # mayble flip if on red. Ideally just based on starting position, though
        return self.odometry.getPose()

    def get_angle(self) -> Rotation2d:
        return self.gyro.get_angle()

    def get_module_positions(self) -> typing.Tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        return (
            self.fl.get_position(),
            self.fr.get_position(),
            self.bl.get_position(),
            self.br.get_position()
        )

    def get_module_states(self) -> typing.Tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        return (
            self.fl.get_state(),
            self.fr.get_state(),
            self.bl.get_state(),
            self.br.get_state()
        )

    def get_speeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.get_module_states())

    """setters"""

    def reset_gyro(self, new_angle: Rotation2d = Rotation2d.fromDegrees(0)) -> InstantCommand:
        return InstantCommand(lambda: self.gyro.reset(new_angle), self)

    def reset_pose(self, pose: Pose2d) -> InstantCommand:
        return InstantCommand(lambda: self.odometry.resetPosition(
            self.get_angle(), self.get_module_positions(), pose), self)

    def _set_drive_idle(self, coast: bool) -> None:
        self.fl.set_drive_idle(coast)
        self.fr.set_drive_idle(coast)
        self.bl.set_drive_idle(coast)
        self.br.set_drive_idle(coast)

    def set_drive_idle(self, coast: bool) -> InstantCommand:
        return InstantCommand(lambda: self._set_drive_idle(coast), self)

    def _set_turn_idle(self, coast: bool) -> None:
        self.fl.set_turn_idle(coast)
        self.fr.set_turn_idle(coast)
        self.bl.set_turn_idle(coast)
        self.br.set_turn_idle(coast)

    def set_turn_idle(self, coast: bool) -> InstantCommand:
        return InstantCommand(lambda: self._set_turn_idle(coast), self)

    def _run_chassis_speeds(self, speeds: ChassisSpeeds, center_of_rotation: Translation2d = Translation2d(0, 0)) -> None:
        states = self.kinematics.toSwerveModuleStates(
            speeds, center_of_rotation)
        self.nettable.putNumber("commandedXVel fps", speeds.vx_fps)
        self.nettable.putNumber("commandedYVel fps", speeds.vy_fps)
        self.nettable.putNumber("commandedThetaVel degps", speeds.omega_dps)

        self._run_module_states(list(states))

    def _run_module_states(self, states: typing.List[SwerveModuleState]) -> None:
        states = self.kinematics.desaturateWheelSpeeds(
            states, self.max_velocity_mps)
        self.fl.set_state(states[0])
        self.fr.set_state(states[1])
        self.bl.set_state(states[2])
        self.br.set_state(states[3])

    def stop(self) -> InstantCommand:
        return InstantCommand(lambda: self._run_chassis_speeds(ChassisSpeeds()), self)

    def drive_joystick(
        self,
        get_x: typing.Callable[[], float],
        get_y: typing.Callable[[], float],
        get_theta: typing.Callable[[], float],
        use_field_oriented: typing.Callable[[], bool]
    ):
        # return StartEndCommand(
        return RunCommand(lambda: self._run_chassis_speeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    applyDeadband(get_x(), 0.1, 1.0) * self.max_velocity_mps,
                    applyDeadband(get_y(), 0.1, 1.0) * self.max_velocity_mps,
                    applyDeadband(get_theta(), 0.1, 1.0) *
                    self.max_angular_velocity.radians(),
                    self.get_angle()
                ) if use_field_oriented() else ChassisSpeeds.fromFieldRelativeSpeeds(
                    applyDeadband(get_x(), 0.1, 1.0) * self.max_velocity_mps,
                    applyDeadband(get_y(), 0.1, 1.0) * self.max_velocity_mps,
                    applyDeadband(get_theta(), 0.1, 1.0) *
                    self.max_angular_velocity.radians(),
                    self.get_angle()
                )
            ),
            self)
        # )
