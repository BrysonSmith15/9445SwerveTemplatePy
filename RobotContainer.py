from math import tan

from wpimath import applyDeadband

import commands2
from commands2.button import CommandXboxController
from commands2 import DeferredCommand, InstantCommand

from subsystems.drivetrain import Drivetrain

from commands.drive_joystick import DriveJoystick


class RobotContainer:
    drive_curve_denominator: float = tan(1) ** 2

    def __init__(self):
        self.drivetrain = Drivetrain()
        self.joystick = CommandXboxController(0)

        self.field_oriented = True

    def get_x(self) -> float:
        raw_x = applyDeadband(self.joystick.getLeftY(), 0.05)
        return (abs(tan(raw_x)) * tan(raw_x)) / self.drive_curve_denominator

    def get_y(self) -> float:
        raw_y = applyDeadband(self.joystick.getLeftX(), 0.05)
        return (abs(tan(raw_y)) * tan(raw_y)) / self.drive_curve_denominator

    def get_omega(self) -> float:
        return applyDeadband(self.joystick.getRightX(), 0.05)

    def set_teleop_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(
            DriveJoystick(
                self.drivetrain,
                self.get_x,
                self.get_y,
                self.get_omega,
                lambda: self.field_oriented,
            )
        )

        def toggle_field_oriented():
            self.field_oriented = not self.field_oriented

        self.joystick.a().onTrue(
            DeferredCommand(lambda: InstantCommand(toggle_field_oriented))
        )

    def get_auto(self) -> commands2.Command:
        return commands2.cmd.none()
