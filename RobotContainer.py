from commands2 import Command, WaitCommand, InstantCommand
from commands2.button import Trigger

from wpilib import Joystick, DriverStation

from subsystems.drivetrain import Drivetrain


class RobotContainer:
    def __init__(self) -> None:
        self.drivetrain = Drivetrain()
        self.driver_controller = Joystick(0)
        # TODO: set the bindings for the controller
        # this sets the motors to idle on disable
        Trigger(DriverStation.isEnabled).onTrue(
            InstantCommand(lambda: self.drivetrain.set_drive_idle(False))
        ).onTrue(self.drivetrain.set_turn_idle(False)).onFalse(
            InstantCommand(lambda: self.drivetrain.set_drive_idle(True))
        ).onFalse(self.drivetrain.set_turn_idle(True))

    def set_teleop_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.drive_joystick(
                self.driver_controller.getX,
                self.driver_controller.getY,
                self.driver_controller.getTwist,
                # this assumes that -1 is resting and 1 is full
                lambda: self.driver_controller.getThrottle() > 0
            )
        )

    def unset_teleop_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(WaitCommand(0))

    def get_auto_command(self) -> Command:
        return Command()
