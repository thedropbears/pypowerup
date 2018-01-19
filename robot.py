#!/usr/bin/env python3

import ctre
import magicbot
import wpilib

from automations.intake import IntakeAutomation
from automations.lifter import LifterAutomation
from components.intake import Intake
from components.lifter import Lifter
from pyswervedrive.swervechassis import SwerveChassis
from pyswervedrive.swervemodule import SwerveModule
from utilities.bno055 import BNO055
from utilities.functions import rescale_js


class Robot(magicbot.MagicRobot):
    # Add magicbot components here using variable annotations.
    # Any components that directly actuate motors should be declared after
    # any higher-level components (automations) that depend on them.

    # Automations
    intake_automation: IntakeAutomation
    lifter_automation: LifterAutomation

    # Actuators
    chassis: SwerveChassis
    intake: Intake
    lifter: Lifter

    module_drive_free_speed: float = 700.

    def createObjects(self):
        """Create non-components here."""

        self.module_a = SwerveModule(  # top left module
            steer_talon=ctre.WPI_TalonSRX(4), drive_talon=ctre.WPI_TalonSRX(13),
            steer_enc_offset=-3893, x_pos=0.3, y_pos=0.3,
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_drive_direction=True, reverse_drive_encoder=True)
        self.module_b = SwerveModule(  # bottom left modulet
            steer_talon=ctre.WPI_TalonSRX(2), drive_talon=ctre.WPI_TalonSRX(9),
            steer_enc_offset=-3543, x_pos=-0.3, y_pos=0.3,
            drive_free_speed=Robot.module_drive_free_speed)
        self.module_c = SwerveModule(  # bottom right modulet
            steer_talon=ctre.WPI_TalonSRX(8), drive_talon=ctre.WPI_TalonSRX(14),
            steer_enc_offset=-1832, x_pos=-0.3, y_pos=-0.3,
            drive_free_speed=Robot.module_drive_free_speed)
        self.module_d = SwerveModule(  # top right modulet
            steer_talon=ctre.WPI_TalonSRX(11), drive_talon=ctre.WPI_TalonSRX(6),
            steer_enc_offset=-1541, x_pos=0.3, y_pos=-0.3,
            drive_free_speed=Robot.module_drive_free_speed)

        # create the imu object
        self.bno055 = BNO055()

        # boilerplate setup for the joystick
        self.joystick = wpilib.Joystick(0)

        self.spin_rate = 5

    def teleopInit(self):
        '''Called when teleop starts; optional'''
        self.bno055.resetHeading()

    def teleopPeriodic(self):
        """
        Process inputs from the driver station here.

        This is run each iteration of the control loop before magicbot components are executed.
        """
        # this is where the joystick inputs get converted to numbers that are sent
        # to the chassis component. we rescale them using the rescale_js function,
        # in order to make their response exponential, and to set a dead zone -
        # which just means if it is under a certain value a 0 will be sent
        # TODO: Tune these constants forvwhatever robot they are on
        vx = -rescale_js(self.joystick.getY(), deadzone=0.05, exponential=1.2, rate=4)
        vy = -rescale_js(self.joystick.getX(), deadzone=0.05, exponential=1.2, rate=4)
        vz = -rescale_js(self.joystick.getZ(), deadzone=0.2, exponential=15.0, rate=self.spin_rate)
        self.chassis.set_inputs(vx, vy, vz)


if __name__ == '__main__':
    wpilib.run(Robot)
