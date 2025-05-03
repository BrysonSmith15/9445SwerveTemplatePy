from wpilib import DataLogManager, TimedRobot
from commands2 import CommandScheduler, Command
import commands2

from RobotContainer import RobotContainer
from subsystems.swerve_module import SwerveModule, ModuleLocation
from subsystems.drivetrain import Drivetrain

import constants

from wpilib import XboxController, SmartDashboard
from wpimath import applyDeadband


class Robot(TimedRobot):
    auto_command: Command = commands2.cmd.none()

    def __init__(self):
        super().__init__()
        self.robotcontainer = RobotContainer()
        self.test_module = SwerveModule(ModuleLocation.BACK_LEFT)
        self.drivetrain = Drivetrain()
        self.joystick = XboxController(0)

    def robotInit(self):
        DataLogManager.start()

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

        self.drivetrain.run_percent(
            applyDeadband(self.joystick.getLeftY(), 0.05),
            applyDeadband(self.joystick.getLeftX(), 0.05),
            applyDeadband(self.joystick.getRightX(), 0.05),
            True,
        )

    def autonomousInit(self):
        self.auto_command = self.robotcontainer.get_auto()
        self.auto_command.schedule()

    def teleopInit(self):
        if self.auto_command is not None and self.auto_command.isScheduled():
            self.auto_command.cancel()
        self.robotcontainer.set_teleop_bindings()
