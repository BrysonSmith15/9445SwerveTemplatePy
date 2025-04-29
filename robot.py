# TODO: insert robot code here
from wpilib import DataLogManager, TimedRobot
from commands2 import CommandScheduler, Command

from RobotContainer import RobotContainer

import constants


class Robot(TimedRobot):
    auto_command: Command

    def __init__(self):
        super().__init__()
        self.robotcontainer = RobotContainer()
        self.test_configs = constants.SwerveConstants()

    def robotInit(self):
        CommandScheduler.getInstance().enable()
        DataLogManager.start()

    def robotPeriodic(self):
        print(self.test_configs.max_speed)

    def autonomousInit(self):
        self.auto_command = self.robotcontainer.get_auto()
        self.auto_command.schedule()

    def teleopInit(self):
        if self.auto_command.is_running():
            self.auto_command.cancel()
        self.robotcontainer.set_teleop_bindings()
