from gryo_base import GyroBase

from navx import AHRS

from wpilib import SerialPort

from wpimath.geometry import Rotation2d

from typing import Self


class NavX(GyroBase):
    def __init__(
        self,
        serial_type: SerialPort,
    ):
        self.hardware = AHRS(serial_type)

    def fromMXP() -> Self:
        return Self.__init__(SerialPort.Port.kMXP)

    def fromUSB(port: int) -> Self:
        if port == 0:
            return Self.__init__(SerialPort.Port.kUSB)
        elif port == 1:
            return Self.__init__(SerialPort.Port.kUSB1)
        elif port == 2:
            return Self.__init__(SerialPort.Port.kUSB1)
        else:
            raise IndexError(
                "The roborio only has 2 usb ports. Your port can only be 1 or 2 [0]"
            )

    def get_angle(self):
        return self.hardware.getRotation2d()

    def reset(self, new_angle: Rotation2d = Rotation2d.fromDegrees(0)) -> None:
        self.hardware.reset()
        self.hardware.setAngleAdjustment(new_angle.degrees())
