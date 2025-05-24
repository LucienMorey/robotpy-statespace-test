import math

import wpilib
from magicbot import feedback
from rev import (
    SparkMax,
    SparkMaxConfig,
)
from wpilib import DutyCycleEncoder
from wpimath import estimator
from wpimath.controller import (
    LinearQuadraticRegulator_2_1,
)
from wpimath.system import LinearSystemLoop_2_1_1
from wpimath.system.plant import DCMotor
from wpimath.trajectory import TrapezoidProfile

from utilities.state_space import single_jointed_arm_system


class Arm:

    DEPLOYED_ANGLE = 0
    RETRACTED_ANGLE = math.pi / 2
    ARM_MOI = 0.181717788

    GEAR_RATIO = 4.0 * 5.0 * (48.0 / 40.0)

    def __init__(self, mech_root: wpilib.MechanismRoot2d) -> None:
        self.intake_ligament = mech_root.appendLigament(
            "intake", length=0.25, angle=90, color=wpilib.Color8Bit(wpilib.Color.kGreen)
        )

        self.motor = SparkMax(0, SparkMax.MotorType.kBrushless)
        self.position_encoder = DutyCycleEncoder(0, math.tau, 0)

        spark_config = SparkMaxConfig()

        spark_config.encoder.positionConversionFactor(math.tau * (1 / Arm.GEAR_RATIO))
        spark_config.encoder.velocityConversionFactor(
            (1 / 60) * math.tau * (1 / Arm.GEAR_RATIO)
        )

        self.motor.configure(
            spark_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.velocity_encoder = self.motor.getEncoder()

        plant = single_jointed_arm_system(DCMotor.NEO(1), self.ARM_MOI, Arm.GEAR_RATIO)

        self.observer = estimator.KalmanFilter_2_1_1(
            plant,
            (
                0.15,
                0.17,
            ),
            (0.005,),
            0.020,
        )

        self.controller = LinearQuadraticRegulator_2_1(
            plant,
            (
                0.01,
                0.5,
            ),
            (12.0,),
            0.020,
        )

        self.loop = LinearSystemLoop_2_1_1(
            plant, self.controller, self.observer, 12.0, 0.020
        )

        self.loop.reset([self.position_observation(), self.velocity_observation()])
        self.loop.setNextR([self.position_observation(), self.velocity_observation()])

        self.motion_profile = TrapezoidProfile(TrapezoidProfile.Constraints(5.0, 5.0))
        self.initial_state = TrapezoidProfile.State(
            self.position_observation(), self.velocity_observation()
        )
        self.tracked_state = self.initial_state
        self.desired_state = TrapezoidProfile.State(Arm.RETRACTED_ANGLE, 0.0)

        self.last_setpoint_update_time = wpilib.Timer.getFPGATimestamp()

    def deploy(self):
        self.desired_state = TrapezoidProfile.State(Arm.DEPLOYED_ANGLE, 0.0)
        self.initial_state = TrapezoidProfile.State(self.position(), self.velocity())
        self.last_setpoint_update_time = wpilib.Timer.getFPGATimestamp()

    def retract(self):
        self.desired_state = TrapezoidProfile.State(Arm.RETRACTED_ANGLE, 0.0)
        self.initial_state = TrapezoidProfile.State(self.position(), self.velocity())
        self.last_setpoint_update_time = wpilib.Timer.getFPGATimestamp()

    def position(self):
        return self.loop.xhat(0)

    @feedback
    def position_observation(self) -> float:
        return self.position_encoder.get()

    def velocity(self) -> float:
        return self.loop.xhat(1)

    @feedback
    def velocity_observation(self) -> float:
        return self.velocity_encoder.getVelocity()

    @feedback
    def state(self):
        return self.loop.xhat()

    @feedback
    def current_input(self):
        return self.loop.U(0)

    def correct_and_predict(self) -> None:
        self.loop.correct([self.position_observation()])

        self.loop.predict(0.020)

        # constrain ourselves if we are going to do damage
        if (
            self.position() > Arm.RETRACTED_ANGLE
            or self.position() < Arm.DEPLOYED_ANGLE
        ):
            self.loop.reset([self.position_observation(), self.velocity_observation()])

    @feedback
    def desired(self):
        return (self.desired_state.position, self.desired_state.velocity)

    @feedback
    def tracked(self):
        return (self.tracked_state.position, self.tracked_state.velocity)

    @feedback
    def initial(self):
        return (self.initial_state.position, self.initial_state.velocity)

    def on_enable(self) -> None:
        self.retract()

    def execute(self) -> None:

        self.tracked_state = self.motion_profile.calculate(
            wpilib.Timer.getFPGATimestamp() - self.last_setpoint_update_time,
            self.initial_state,
            self.desired_state,
        )

        self.loop.setNextR([self.tracked_state.position, self.tracked_state.velocity])

        self.correct_and_predict()

        self.motor.setVoltage(self.loop.U(0))

    def periodic(self) -> None:
        self.intake_ligament.setAngle(math.degrees(self.position()))
