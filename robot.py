#!/usr/bin/env python3

import ctre
import magicbot
import wpilib


class Robot(magicbot.MagicRobot):
    # Add magicbot components here using variable annotations.

    def createObjects(self):
        """Create non-components here."""
        pass

    def teleopPeriodic(self):
        """This is run each iteration of the control loop before magicbot components are executed."""
        pass


if __name__ == '__main__':
    wpilib.run(Robot)
