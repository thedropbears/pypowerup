#!/usr/bin/env python3

import magicbot
import wpilib

from automations.containment import Containment
from components.intake import Intake
from ctre import WPI_TalonSRX, CANifier
from automations.lifter import LifterAutomation

class Robot(magicbot.MagicRobot):
    # Add magicbot components here using variable annotations.
    # Any components that directly actuate motors should be declared after
    # any higher-level components (automations) that depend on them.

    # Automations
    containment: Containment
    lifter_automation: LifterAutomation

    # Actuators
    intake: Intake

    def createObjects(self):
        """Create non-components here."""
        self.xbox = wpilib.XboxController(0)
        """This is to state what channel our xbox controller is on"""
        self.intake_motor1 = WPI_TalonSRX(1)
        """This controls the front section of the intake mechanism, This controls two motors."""
        self.intake_motor2 = WPI_TalonSRX(2)
        """This controls the back section of the intake mechanism, this controls two motors."""
        self.clamp_arm_left = wpilib.Solenoid(0)
        self.clamp_arm_right = wpilib.Solenoid(1)
        """This controls the arm in the back section of the intake mechanism"""
        self.intake_kicker = wpilib.Solenoid(3)
        """This controls the kicker in the back section of the intake mechanism"""
        self.limit_switch = CANifier(4)

    def teleopInit(self):
        '''Called when teleop starts; optional'''
        self.intake.intake_clamp(False)
        self.intake.intake_push(False)

    def teleopPeriodic(self):
        """
        Process inputs from the driver station here.

        This is run each iteration of the control loop before magicbot
        components are executed.
        """
        # this is where the joystick inputs get converted to numbers that
        # are sent to the chassis component. we rescale them using the
        # rescale_js function, in order to make their response exponential,
        # and to set a dead zone which just means if it is under a certain
        # value a 0 will be sent
        # TODO: Tune these constants for whatever robot they are on
        # self.intake.intake_arm(self.xbox.getBButton())

        if self.xbox.getAButtonReleased():
            self.test_automation.engage()


if __name__ == '__main__':
    wpilib.run(Robot)
