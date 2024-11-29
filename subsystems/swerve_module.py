import math

from rev import CANSparkMax, CANSparkLowLevel

from phoenix6.hardware import CANcoder
from phoenix6.configs import MagnetSensorConfigs
from phoenix6.signals import SensorDirectionValue, AbsoluteSensorRangeValue

from commands2 import Subsystem

from ntcore import NetworkTableInstance, EventFlags, Event

from wpimath.kinematics import SwerveModuleState
from wpimath.units import inchesToMeters
from wpimath.geometry import Rotation2d


class SwerveModule(Subsystem):
    def __init__(self, name: str, drive_id: int, turn_id: int, cancoder_id: int):
        # cancoder
        self.cancoder = CANcoder(cancoder_id)
        self.cancoder.configurator.apply(
            MagnetSensorConfigs()
            .with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE)
            .with_absolute_sensor_range(AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF)
        )

        self.name = name
        self.setName(name)

        # drive
        self.drive_motor = CANSparkMax(
            drive_id, CANSparkLowLevel.MotorType.kBrushless)

        self.drive_encoder = self.drive_motor.getEncoder()
        self.drive_encoder.setVelocityConversionFactor(1 / 8.14)
        self.drive_encoder.setPositionConversionFactor(1 / 8.14)
        self.drive_encoder.setPosition(0)

        self.drive_pid = self.drive_motor.getPIDController()
        self.drive_pid.setP(0.01)
        self.drive_pid.setI(0)
        self.drive_pid.setD(0.001)

        # turn
        self.turn_motor = CANSparkMax(
            turn_id, CANSparkLowLevel.MotorType.kBrushless)

        self.turn_encoder = self.turn_motor.getEncoder()
        self.turn_encoder.setVelocityConversionFactor(1 / 150.7)
        self.turn_encoder.setPositionConversionFactor(1 / 150.7)
        self.turn_encoder.setPosition(
            self.cancoder.get_absolute_position().value_as_double
        )

        self.turn_pid = self.turn_motor.getPIDController()
        self.turn_pid.setP(0.01)
        self.turn_pid.setI(0)
        self.turn_pid.setD(0.001)

        # networktables
        self.nettable = NetworkTableInstance.getDefault().getTable(
            f"swerve/{self.name}"
        )

        self.nettable.addListener(
            "driveP", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener(
            "driveI", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener(
            "driveD", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener(
            "turnD", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener(
            "turnI", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener(
            "turnP", EventFlags.kValueAll, self._nt_pid_listener)

        # this is the real place to set the pid values
        self.nettable.setDefaultNumber("driveP", 0.01)
        self.nettable.setDefaultNumber("driveI", 0.01)
        self.nettable.setDefaultNumber("driveD", 0.01)
        self.nettable.setDefaultNumber("turnP", 0.01)
        self.nettable.setDefaultNumber("turnI", 0.01)
        self.nettable.setDefaultNumber("turnD", 0.01)

    def _nt_pid_listener(self, _nt, key: str, event: Event):
        try:
            var = key[-1]
            if key.startswith("drive"):
                if var == "P":
                    self.drive_pid.setP(event.data.value.value())
                elif var == "I":
                    self.drive_pid.setI(event.data.value.value())
                elif var == "D":
                    self.drive_pid.setD(event.data.value.value())
            elif key.startswith("turn"):
                if var == "P":
                    self.turn_pid.setP(event.data.value.value())
                elif var == "I":
                    self.turn_pid.setI(event.data.value.value())
                elif var == "D":
                    self.turn_pid.setD(event.data.value.value())
            else:
                print(f"failed at {key}")
        except Exception:
            pass

    def get_vel(self) -> float:
        """
        return the velocity in meters per second

        circumfrence of a circle is 2 * pi * r.
        The wheel radius of the mk4i is 2in.
        """
        return self.drive_encoder.getVelocity() * 2 * math.pi * inchesToMeters(2)

    def get_angle(self) -> Rotation2d:
        """return the angle of the swerve module as a Rotation2d"""
        return Rotation2d.fromRotations(self.turn_encoder.getPosition())

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_vel(), self.get_angle())

    def set_state(self, commanded_state: SwerveModuleState) -> SwerveModuleState:
        # optimize the new state
        commanded_state = SwerveModuleState.optimize(
            commanded_state, self.get_angle())
        # set the turn pid in rotations
        # (degrees % 360) / 360 => (wrap the angle from [0, 360]) / (angles per rotation)
        # above => convert degress to rotations
        self.turn_pid.setReference(
            self._rotation2d_to_rotations(commanded_state.angle),
            CANSparkLowLevel.ControlType.kPosition,
        )

        # print(commanded_state.speed_fps / (2 * math.pi * (1 / 6)))
        self.drive_pid.setReference(
            commanded_state.speed_fps / (2 * math.pi * (1 / 6)), CANSparkLowLevel.ControlType.kVelocity)

    def _rotation2d_to_rotations(self, angle: Rotation2d) -> float:
        return (angle.degrees() % 360) / 360
