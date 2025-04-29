from enum import Enum

from commands2 import Subsystem

from phoenix6.hardware import TalonFX, cancoder
import phoenix6


class ModuleLocation(Enum):
    FRONT_LEFT = (0,)
    FRONT_RIGHT = (1,)
    BACK_LEFT = (2,)
    BACK_RIGHT = (3,)


class SwerveModule(Subsystem):
    def __init__(self, location: ModuleLocation):
        super().__init__()
        self.setName("Swerve Module")
