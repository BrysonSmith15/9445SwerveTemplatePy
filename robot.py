# TODO: insert robot code here
from wpilib import DataLogManager, TimedRobot
from commands2 import CommandScheduler, Command
import commands2

from ntcore import NetworkTableInstance

from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d

from RobotContainer import RobotContainer
from subsystems.swerve_module import SwerveModule, ModuleLocation

import constants


class Robot(TimedRobot):
    auto_command: Command

    def __init__(self):
        super().__init__()
        self.robotcontainer = RobotContainer()
        self.test_module = SwerveModule(ModuleLocation.BACK_LEFT)
        self.test_module.set_state(SwerveModuleState())
        self.pub = (
            NetworkTableInstance.getDefault()
            .getStructArrayTopic("swerve/test", SwerveModuleState)
            .publish()
        )
        self.real_pub = (
            NetworkTableInstance.getDefault()
            .getStructArrayTopic("swerve/test2", SwerveModuleState)
            .publish()
        )
        self.auto_command = commands2.cmd.none()
        self.setpoint = 0

    def robotInit(self):
        DataLogManager.start()

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()
        self.test_module.set_state(
            SwerveModuleState(
                1,
                Rotation2d.fromDegrees(self.setpoint),
                # Rotation2d.fromDegrees(self.test_module.setpoint.angle.degrees() + 1),
            )
        )

        self.pub.set(
            [
                self.test_module.setpoint,
                self.test_module.setpoint,
                self.test_module.setpoint,
                self.test_module.setpoint,
            ]
        )
        self.real_pub.set(
            [
                self.test_module.get_state(),
                self.test_module.get_state(),
                self.test_module.get_state(),
                self.test_module.get_state(),
            ]
        )

    def autonomousInit(self):
        self.auto_command = self.robotcontainer.get_auto()
        self.auto_command.schedule()

    def teleopInit(self):
        if self.auto_command is not None and self.auto_command.isScheduled():
            self.auto_command.cancel()
        self.robotcontainer.set_teleop_bindings()
        self.setpoint += 90
