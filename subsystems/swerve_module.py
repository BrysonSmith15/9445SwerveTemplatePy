import math

from rev import CANSparkMax, CANSparkLowLevel

from phoenix6.hardware import CANcoder
from phoenix6.configs import MagnetSensorConfigs
from phoenix6.signals import SensorDirectionValue, AbsoluteSensorRangeValue

from commands2 import Subsystem

from ntcore import NetworkTableInstance, EventFlags, Event

from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.units import inchesToMeters
from wpimath.geometry import Rotation2d


class SwerveModule(Subsystem):
    def __init__(
        self,
        name: str,
        drive_id: int,
        turn_id: int,
        cancoder_id: int,
        drive_inverted: bool,
        turn_inverted: bool,
    ):
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
        self.drive_motor.setInverted(drive_inverted)

        self.drive_encoder = self.drive_motor.getEncoder()
        self.drive_encoder.setMeasurementPeriod(20)
        self.drive_encoder.setAverageDepth(2)
        self.drive_encoder.setVelocityConversionFactor((2 * math.pi * inchesToMeters(2)) / (8.14 * 60))
        self.drive_encoder.setPositionConversionFactor((2 * math.pi) * inchesToMeters(2) / 8.14)
        self.drive_encoder.setPosition(0)

        self.drive_pid = self.drive_motor.getPIDController()
        self.drive_pid.setP(0.01)
        self.drive_pid.setI(0)
        self.drive_pid.setD(0.001)
        self.drive_motor.setClosedLoopRampRate(0.20)
        self.drive_pid.setOutputRange(-1,1)

        self.drive_pid.setFeedbackDevice(self.drive_encoder)

        # turn
        self.turn_motor = CANSparkMax(
            turn_id, CANSparkLowLevel.MotorType.kBrushless)
        self.turn_motor.setInverted(turn_inverted)

        self.turn_encoder = self.turn_motor.getEncoder()
        self.turn_encoder.setVelocityConversionFactor(1 / 150.7)
        self.turn_encoder.setPositionConversionFactor(1 / (150.7 * 60))
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
        self.nettable.setDefaultNumber("driveI", 0.00)
        self.nettable.setDefaultNumber("driveD", 0.01)
        self.nettable.setDefaultNumber("turnP", 0.01)
        self.nettable.setDefaultNumber("turnI", 0.00)
        self.nettable.setDefaultNumber("turnD", 0.01)

        """misc variables"""
        # these variables are for doing something every mod_value runs of periodic
        self.mod_counter = 0
        self.mod_value = 50

    def periodic(self) -> None:
        self.mod_counter = (self.mod_counter + 1) % self.mod_value
        if self.mod_counter == 0:
            self.turn_encoder.setPosition(
                self.cancoder.get_absolute_position().value_as_double)

        self.nettable.putNumber("State/velocity (mps)", self.get_vel())
        self.nettable.putNumber("State/angle (deg)",
                                self.get_angle().degrees())

    def get_vel(self) -> float:
        """
        return the velocity in meters per second

        circumfrence of a circle is 2 * pi * r.
        The wheel radius of the mk4i is 2in.
        """
        return self.drive_encoder.getVelocity()

    def get_distance(self) -> float:
        """return the distance driven by the swerve module since powered on"""
        return self.drive_encoder.getPosition()

    def get_angle(self) -> Rotation2d:
        """return the angle of the swerve module as a Rotation2d"""
        return Rotation2d.fromDegrees(self.turn_encoder.getPosition() * 180 / math.pi)

    def get_state(self) -> SwerveModuleState:
        """return the velocity and angle of the swerve module"""
        return SwerveModuleState(self.get_vel(), self.get_angle())

    def get_position(self) -> SwerveModulePosition:
        """get the distance driven and angle of the module"""
        return SwerveModulePosition(self.get_distance(), self.get_angle())

    def set_drive_idle(self, coast: bool) -> None:
        self.drive_motor.setIdleMode(
            CANSparkMax.IdleMode.kCoast if coast else CANSparkMax.IdleMode.kBrake)

    def set_turn_idle(self, coast: bool) -> None:
        self.turn_motor.setIdleMode(
            CANSparkMax.IdleMode.kCoast if coast else CANSparkMax.IdleMode.kBrake)

    def set_state(self, commanded_state: SwerveModuleState) -> SwerveModuleState:
        """command the swerve module to an angle and speed"""
        # optimize the new state
        commanded_state = SwerveModuleState.optimize(
            commanded_state, self.get_angle())
        # set the turn pid in rotations
        # (degrees % 360) / 360 => (wrap the angle from [0, 360]) / (angles per rotation)
        # above => convert degrees to rotations
        self.turn_pid.setReference(
            self._rotation2d_to_rotations(commanded_state.angle),
            CANSparkLowLevel.ControlType.kPosition,
        )

        """
        cosine optimization - make the wheel slower when pointed the wrong direction
        note that there in no abs over cos because cos(-x) == cos(x)
        """
        cos_optimizer = (commanded_state.angle - self.get_angle()).cos()
        self.nettable.putNumber("thing", commanded_state.speed)

        self.drive_pid.setReference(
            (commanded_state.speed * cos_optimizer,
            CANSparkLowLevel.ControlType.kVelocity,
        )

    def _rotation2d_to_rotations(self, angle: Rotation2d) -> float:
        return (angle.degrees() % 360) / 360

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
