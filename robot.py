import magicbot
import wpilib

from components.arm import Arm


class MyRobot(magicbot.MagicRobot):
    arm: Arm

    def createObjects(self) -> None:
        self.mech = wpilib.Mechanism2d(2, 2)
        wpilib.SmartDashboard.putData("Mech2d", self.mech)
        self.mech_root = self.mech.getRoot("Intake", 1.5, 0.1)

        self.gamepad = wpilib.XboxController(0)

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:

        if self.gamepad.getAButton():
            self.arm.deploy()
        if self.gamepad.getYButton():
            self.arm.retract()

    def disabledPeriodic(self) -> None:
        self.arm.correct_and_predict()

    def robotPeriodic(self) -> None:
        super().robotPeriodic()
        self.arm.periodic()
