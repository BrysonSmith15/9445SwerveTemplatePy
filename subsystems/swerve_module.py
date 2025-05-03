from enum import Enum
from math import pi

from commands2 import Subsystem

from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
from wpimath.units import meters_per_second, meters

from ntcore import NetworkTableInstance

from phoenix6.hardware import TalonFX
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.configs import FeedbackConfigs, MagnetSensorConfigs
from phoenix6.controls import VelocityDutyCycle, PositionDutyCycle

from constants import ModuleConstants, SwerveConstants
from errors import Error


class ModuleLocation(Enum):
    FRONT_LEFT = (0,)
    FRONT_RIGHT = (1,)
    BACK_LEFT = (2,)
    BACK_RIGHT = (3,)


class SwerveModule(Subsystem):
    def __init__(self, location: ModuleLocation):
        super().__init__()
        swerve_consts = SwerveConstants()

        consts: ModuleConstants = None
        if location == ModuleLocation.FRONT_LEFT:
            consts = swerve_consts.front_left
            self.setName("Swerve Module FL")
        elif location == ModuleLocation.FRONT_RIGHT:
            consts = swerve_consts.front_right
            self.setName("Swerve Module FR")
        elif location == ModuleLocation.BACK_LEFT:
            consts = swerve_consts.back_left
            self.setName("Swerve Module BL")
        elif location == ModuleLocation.BACK_RIGHT:
            consts = swerve_consts.back_right
            self.setName("Swerve Module BR")
        else:
            Error(
                ValueError(
                    f"There was an invalid module location given to SwerveModule {location}"
                )
            )
            # this may be bad. If in a real match, the code will report an error, then keep running. What is the consts value there?
            consts = SwerveConstants.front_left
            self.setName("Swerve Module ERROR")

        self.nettable = NetworkTableInstance.getDefault().getTable(
            f"/swerve/modules/{self.getName()}"
        )

        self.drive_motor = TalonFX(consts.drive_id, SwerveConstants.canbus)
        self.turn_motor = TalonFX(consts.turn_id, SwerveConstants.canbus)
        self.cancoder = CANcoder(consts.cancoder_id, SwerveConstants.canbus)

        self.cancoder.configurator.apply(
            swerve_consts.cancoder_config.with_magnet_sensor(
                MagnetSensorConfigs().with_magnet_offset(consts.cancoder_offset)
            )
        )
        self.drive_motor.configurator.apply(swerve_consts.drive_config)
        self.turn_motor.configurator.apply(
            swerve_consts.turn_config.with_feedback(
                FeedbackConfigs()
                .with_feedback_remote_sensor_id(consts.turn_id)
                .with_rotor_to_sensor_ratio(SwerveConstants.turn_ratio)
            )
        )

        self.setpoint = SwerveModuleState()
        try_angle = self.turn_motor.get_position()
        i = 0
        while (not try_angle.is_all_good) and i < 10:
            try_angle = self.turn_motor.get_position()
        self.last_good_angle = try_angle.value

        self.commanded_pub = self.nettable.getStructTopic(
            "State/Commanded", SwerveModuleState
        ).publish()
        self.actual_pub = self.nettable.getStructTopic(
            "State/Actual", SwerveModuleState
        ).publish()

        self.set_state(self.setpoint)

        self.cancoder_sim = self.cancoder.sim_state

    def periodic(self):
        self.drive_motor.set_control(
            VelocityDutyCycle(
                self.setpoint.speed
                / SwerveConstants.drive_ratio
                / SwerveConstants.wheel_radius
                / (2 * pi)
            )
        )
        self.turn_motor.set_control(
            PositionDutyCycle(self.setpoint.angle.degrees() / 360)
        )

        self.actual_pub.set(
            SwerveModuleState(
                self.get_speed(),
                self.get_angle(),
            ),
        )

        self.nettable.putNumber("State/Drive Out", self.drive_motor.get())
        self.nettable.putNumber("State/Turn Out", self.turn_motor.get())
        self.nettable.putNumber("State/Drive Velocity", self.get_speed())
        self.nettable.putNumber(
            "State/Turn Velocity", self.turn_motor.get_velocity().value
        )
        self.nettable.putNumber("State/drive distance", self.get_distance())

        # return super().periodic()

    def simulationPeriodic(self):
        # Drive Motor Position and Velocity
        driveRps = 6000 * self.drive_motor.get() * SwerveConstants.drive_ratio
        # driveRps = self.drive_motor.get_velocity().value
        self.drive_motor.sim_state.set_rotor_velocity(driveRps)
        self.drive_motor.sim_state.add_rotor_position(driveRps * 0.02)

        # Turn Motor Position and Velocity
        turnRps = 6000 * self.turn_motor.get() * SwerveConstants.turn_ratio
        # turnRps = self.turn_motor.get_velocity().value * SwerveConstants.turn_ratio
        self.turn_motor.sim_state.set_rotor_velocity(turnRps)
        self.turn_motor.sim_state.add_rotor_position(turnRps * 0.02)

        # CANcoder Velocity and Position
        canRps = turnRps * SwerveConstants.turn_ratio
        self.cancoder_sim.set_velocity(canRps)
        self.cancoder_sim.add_position(canRps * 0.02)
        self.nettable.putNumber("CanTurnRPS", canRps)

    def get_speed(self) -> meters_per_second:
        return (
            self.drive_motor.get_velocity().value
            * (SwerveConstants.drive_ratio)  # * SwerveConstants.wheel_radius)
            * (2 * pi * SwerveConstants.wheel_radius)
        )

    def get_angle(self) -> Rotation2d:
        try_angle = self.cancoder.get_absolute_position().wait_for_update(0.1)
        if try_angle.is_all_good():
            self.last_good_angle = try_angle.value
            return Rotation2d.fromRotations(self.turn_motor.get_position().value)
        else:
            return Rotation2d.fromRotations(self.last_good_angle)

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_speed(), self.get_angle())

    def get_distance(self) -> meters:
        return (
            self.drive_motor.get_position().value
            * (SwerveConstants.drive_ratio * SwerveConstants.wheel_radius)
            * (2 * pi)
        )

    def set_state(self, state: SwerveModuleState) -> None:
        state.optimize(Rotation2d.fromRotations(self.turn_motor.get_position().value))
        state.cosineScale(
            Rotation2d.fromRotations(self.turn_motor.get_position().value)
        )

        self.setpoint = state
        self.commanded_pub.set(state)
