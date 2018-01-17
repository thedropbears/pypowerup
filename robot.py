#!/usr/bin/env python3

import ctre
import magicbot
import wpilib

from automations.intake import IntakeAutomation
from automations.lifter import LifterAutomation
from components.intake import Intake
from components.lifter import Lifter


class Robot(magicbot.MagicRobot):
    # Add magicbot components here using variable annotations.
    # Any components that directly actuate motors should be declared after
    # any higher-level components (automations) that depend on them.

    # Automations
    intake_automation: IntakeAutomation
    lifter_automation: LifterAutomation

    # Actuators
    intake: Intake
    lifter: Lifter

    def createObjects(self):
        """Create non-components here."""
        pass

    def teleopPeriodic(self):
        """
        Process inputs from the driver station here.

        This is run each iteration of the control loop before magicbot components are executed.
        """
        pass


if __name__ == '__main__':
    wpilib.run(Robot)
