import math

from rev import SparkBaseConfig, SparkMax, SparkLowLevel, SparkMaxConfig, SparkBase

from phoenix6.hardware import CANcoder
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.signals import SensorDirectionValue

from commands2 import Subsystem

from ntcore import NetworkTableInstance, EventFlags, Event, ValueEventData

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
            .with_absolute_sensor_discontinuity_point(1)
        )

        self.name = name
        self.setName(name)

        # drive
        self.drive_motor = SparkMax(drive_id, SparkLowLevel.MotorType.kBrushless)
        self.drive_encoder = self.drive_motor.getEncoder()
        self.drive_pid = self.drive_motor.getClosedLoopController()

        self.drive_motor_config = SparkMaxConfig()
        self.drive_motor_config.inverted(
            drive_inverted
        ).encoder.velocityConversionFactor(
            (2 * math.pi * inchesToMeters(2)) / (8.14 * 60)
        ).positionConversionFactor(
            (2 * math.pi) * inchesToMeters(2) / 8.14
        )

        self.drive_motor_config.closedLoop.P(0.01).I(0).D(0.001)

        self.drive_encoder.setPosition(0)

        self.drive_motor.configure(
            self.drive_motor_config,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        # turn
        self.turn_motor = SparkMax(turn_id, SparkLowLevel.MotorType.kBrushless)
        self.turn_pid = self.turn_motor.getClosedLoopController()
        self.turn_encoder = self.turn_motor.getEncoder()
        self.turn_motor_config = SparkMaxConfig()
        self.turn_motor_config.inverted(turn_inverted).encoder.positionConversionFactor(
            1 / 150.7
        ).velocityConversionFactor(1 / (150.7 * 60))

        self.turn_encoder.setPosition(
            self.cancoder.get_absolute_position().value_as_double
        )

        self.turn_motor_config.closedLoop.P(0.01).I(0).D(0.001)

        self.turn_motor.configure(
            self.turn_motor_config,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        # networktables
        self.nettable = NetworkTableInstance.getDefault().getTable(
            f"swerve/{self.name}"
        )

        self.nettable.addListener("driveP", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener("driveI", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener("driveD", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener("turnD", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener("turnI", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener("turnP", EventFlags.kValueAll, self._nt_pid_listener)

        self.nettable.setDefaultNumber(
            "driveP", self.drive_motor.configAccessor.closedLoop.getP()
        )
        self.nettable.setDefaultNumber(
            "driveI", self.drive_motor.configAccessor.closedLoop.getI()
        )
        self.nettable.setDefaultNumber(
            "driveD", self.drive_motor.configAccessor.closedLoop.getD()
        )
        self.nettable.setDefaultNumber(
            "turnP", self.turn_motor.configAccessor.closedLoop.getP()
        )
        self.nettable.setDefaultNumber(
            "turnI", self.turn_motor.configAccessor.closedLoop.getI()
        )
        self.nettable.setDefaultNumber(
            "turnD", self.turn_motor.configAccessor.closedLoop.getD()
        )

        """misc variables"""
        # these variables are for doing something every mod_value runs of periodic
        self.mod_counter = 0
        self.mod_value = 50

    def periodic(self) -> None:
        self.mod_counter = (self.mod_counter + 1) % self.mod_value
        if self.mod_counter == 0:
            self.turn_encoder.setPosition(
                self.cancoder.get_absolute_position().value_as_double
            )

        self.nettable.putNumber("State/velocity (mps)", self.get_vel())
        self.nettable.putNumber("State/angle (deg)", self.get_angle().degrees())

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
        self.drive_motor_config.setIdleMode(
            SparkBaseConfig.IdleMode.kCoast
            if coast
            else SparkBaseConfig.IdleMode.kBrake
        )
        self._configure_drive()

    def set_turn_idle(self, coast: bool) -> None:
        self.turn_motor_config.setIdleMode(
            SparkBaseConfig.IdleMode.kCoast
            if coast
            else SparkBaseConfig.IdleMode.kBrake
        )
        self._configure_turn()

    def set_state(self, commanded_state: SwerveModuleState) -> None:
        """command the swerve module to an angle and speed"""
        # optimize the new state
        # this just mutates commanded_state in place
        commanded_state.optimize(self.get_angle())
        # set the turn pid in rotations
        # (degrees % 360) / 360 => (wrap the angle from [0, 360]) / (angles per rotation)
        # above => convert degrees to rotations
        self.turn_pid.setReference(
            self._rotation2d_to_rotations(commanded_state.angle),
            SparkLowLevel.ControlType.kPosition,
        )

        """
        cosine optimization - make the wheel slower when pointed the wrong direction
        note that there in no abs over cos because cos(-x) == cos(x)
        """
        cos_optimizer = (commanded_state.angle - self.get_angle()).cos()
        self.nettable.putNumber("thing", commanded_state.speed)

        self.drive_pid.setReference(
            (commanded_state.speed * cos_optimizer),
            SparkLowLevel.ControlType.kVelocity,
        )

    def _rotation2d_to_rotations(self, angle: Rotation2d) -> float:
        return (angle.degrees() % 360) / 360

    def _nt_pid_listener(self, _nt, key: str, event: Event):
        try:
            var = key[-1]
            if isinstance((v := event.data), ValueEventData):
                if key.startswith("drive"):
                    if var == "P":
                        self.drive_motor_config.closedLoop.P(v.value.value())
                    elif var == "I":
                        self.drive_motor_config.closedLoop.I(v.value.value())
                    elif var == "D":
                        self.drive_motor_config.closedLoop.D(v.value.value())
                    self._configure_drive()
                elif key.startswith("turn"):
                    if var == "P":
                        self.turn_motor_config.closedLoop.P(v.value.value())
                    elif var == "I":
                        self.turn_motor_config.closedLoop.I(v.value.value())
                    elif var == "D":
                        self.turn_motor_config.closedLoop.D(v.value.value())
                    self._configure_turn()
                else:
                    print(f"failed at {key}")
        except Exception:
            pass

    def _configure_drive(self) -> None:
        self.drive_motor.configure(
            self.drive_motor_config,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )

    def _configure_turn(self) -> None:
        self.turn_motor.configure(
            self.turn_motor_config,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )
