from __future__ import annotations

import typing

import rev
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import (
    DutyCycleEncoderSim,
    SingleJointedArmSim,
)
from wpimath.system.plant import DCMotor

from components.arm import Arm

if typing.TYPE_CHECKING:
    from robot import MyRobot


class SparkArmSim:
    def __init__(self, mech_sim: SingleJointedArmSim, motor_sim: rev.SparkSim) -> None:
        self.mech_sim = mech_sim
        self.motor_sim = motor_sim
        self.motor_encoder_sim = self.motor_sim.getRelativeEncoderSim()

    def update(self, dt: float) -> None:
        vbus = self.motor_sim.getBusVoltage()
        self.mech_sim.setInputVoltage(self.motor_sim.getAppliedOutput() * vbus)
        self.mech_sim.update(dt)
        self.motor_sim.iterate(self.mech_sim.getVelocity(), vbus, dt)
        self.motor_encoder_sim.iterate(self.mech_sim.getVelocity(), dt)


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller

        # Intake arm simulation
        intake_arm_gearbox = DCMotor.NEO(1)
        self.intake_motor = rev.SparkMaxSim(robot.arm.motor, intake_arm_gearbox)
        self.intake_position_encoder = DutyCycleEncoderSim(robot.arm.position_encoder)
        self.intake_arm = SparkArmSim(
            SingleJointedArmSim(
                intake_arm_gearbox,
                Arm.GEAR_RATIO,
                moi=Arm.ARM_MOI,
                armLength=0.22,
                minAngle=Arm.DEPLOYED_ANGLE,
                maxAngle=Arm.RETRACTED_ANGLE,
                simulateGravity=True,
                startingAngle=Arm.RETRACTED_ANGLE,
            ),
            self.intake_motor,
        )

    def update_sim(self, now: float, tm_diff: float) -> None:

        # Update intake arm simulation
        self.intake_arm.update(tm_diff)
        self.intake_position_encoder.set(self.intake_arm.mech_sim.getAngle())
