#!/usr/bin/env python3

import magicbot
import wpilib
from networktables import NetworkTables
import numpy as np
from automations.intake import IntakeAutomation
<<<<<<< HEAD
<<<<<<< HEAD
from automations.lifter import LifterAutomation
from automations.motion import ChassisMotion
from components.intake import Intake
from components.lifter import Lifter
from components.vision import Vision
from pyswervedrive.swervechassis import SwerveChassis
from pyswervedrive.swervemodule import SwerveModule
from utilities.bno055 import BNO055
from utilities.functions import rescale_js
=======
from components.intake import Intake
from ctre import WPI_TalonSRX
>>>>>>> d3c8d1f4aee24fe9a79a7a5a451e5bdb838412a7

import math
=======
from components.intake import Intake
from ctre import WPI_TalonSRX
>>>>>>> d3c8d1f4aee24fe9a79a7a5a451e5bdb838412a7


class Robot(magicbot.MagicRobot):
    # Add magicbot components here using variable annotations.
    # Any components that directly actuate motors should be declared after
    # any higher-level components (automations) that depend on them.

    ds: wpilib.DriverStation

    vision: Vision

    # Automations
    motion: ChassisMotion
    intake_automation: IntakeAutomation

    # Actuators
    intake: Intake
<<<<<<< HEAD
    lifter: Lifter

    module_drive_free_speed: float = 780.
    length: float = 0.88

    def createObjects(self):
        """Create non-components here."""

        self.module_a = SwerveModule(  # top left module
            steer_talon=ctre.WPI_TalonSRX(2), drive_talon=ctre.WPI_TalonSRX(9),
            steer_enc_offset=-2055, x_pos=0.31, y_pos=0.26,
            drive_free_speed=Robot.module_drive_free_speed)
        self.module_b = SwerveModule(  # bottom left modulet
            steer_talon=ctre.WPI_TalonSRX(11), drive_talon=ctre.WPI_TalonSRX(13),
            steer_enc_offset=-2583, x_pos=-0.31, y_pos=0.26,
            drive_free_speed=Robot.module_drive_free_speed)
        self.module_c = SwerveModule(  # bottom right modulet
            steer_talon=ctre.WPI_TalonSRX(8), drive_talon=ctre.WPI_TalonSRX(6),
            steer_enc_offset=-1665, x_pos=-0.31, y_pos=-0.26,
            drive_free_speed=Robot.module_drive_free_speed)
        self.module_d = SwerveModule(  # top right modulet
            steer_talon=ctre.WPI_TalonSRX(4), drive_talon=ctre.WPI_TalonSRX(14),
            steer_enc_offset=-286, x_pos=0.31, y_pos=-0.26,
            drive_free_speed=Robot.module_drive_free_speed)

        # create the imu object
        self.bno055 = BNO055()

        # boilerplate setup for the joystick
        self.joystick = wpilib.Joystick(0)

        self.cube_switch = wpilib.DigitalInput(0)

        self.spin_rate = 5
=======

    def createObjects(self):
        """Create non-components here."""
        """This is to state what channel our xbox controller is on"""
        self.xbox = wpilib.XboxController(0)
        """This controls the front section of the intake mechanism, This controls two motors."""
        self.intake_motor1 = WPI_TalonSRX(1)
        """This controls the back section of the intake mechanism, this controls two motors."""
        self.intake_motor2 = WPI_TalonSRX(2)
        """This controls the left arm in the containment mechanism"""
        self.clamp_arm_left = wpilib.Solenoid(0)
        """This controls the right arm in the containment mechanism"""
        self.clamp_arm_right = wpilib.Solenoid(1)
        """This controls the kicker in the back section of the intake mechanism"""
        self.intake_kicker = wpilib.Solenoid(2)
        self.extension_arm_left = wpilib.Solenoid(3)
        self.extension_arm_right = wpilib.Solenoid(4)
        """This is the limit switch at the back of the containment section"""
        self.limit_switch = wpilib.DigitalInput(0)
>>>>>>> d3c8d1f4aee24fe9a79a7a5a451e5bdb838412a7

    def createObjects(self):
        """Create non-components here."""
        """This is to state what channel our xbox controller is on"""
        self.xbox = wpilib.XboxController(0)
        """This controls the front section of the intake mechanism, This controls two motors."""
        self.intake_motor1 = WPI_TalonSRX(1)
        """This controls the back section of the intake mechanism, this controls two motors."""
        self.intake_motor2 = WPI_TalonSRX(2)
        """This controls the left arm in the containment mechanism"""
        self.clamp_arm_left = wpilib.Solenoid(0)
        """This controls the right arm in the containment mechanism"""
        self.clamp_arm_right = wpilib.Solenoid(1)
        """This controls the kicker in the back section of the intake mechanism"""
        self.intake_kicker = wpilib.Solenoid(2)
        self.extension_arm_left = wpilib.Solenoid(3)
        self.extension_arm_right = wpilib.Solenoid(4)
        """This is the limit switch at the back of the containment section"""
        self.limit_switch = wpilib.DigitalInput(0)

        NetworkTables.initialize()
        self.sd = NetworkTables.getTable("SmartDashboard")

    def teleopInit(self):
        '''Called when teleop starts; optional'''
<<<<<<< HEAD
        self.motion.enabled = False
        self.chassis.set_inputs(0, 0, 0)
=======
>>>>>>> d3c8d1f4aee24fe9a79a7a5a451e5bdb838412a7
        self.intake.intake_clamp(False)
        self.intake.intake_push(False)
        self.extensions(True)
        self.lift_motor = WPI_TalonSRX(0)

    def teleopPeriodic(self):
        """
        Process inputs from the driver station here.

        This is run each iteration of the control loop before magicbot
        components are executed.
        """
<<<<<<< HEAD
        if self.joystick.getRawButtonPressed(10):
            self.chassis.odometry_x = 0.0
            self.chassis.odometry_y = 0.0
            self.motion.set_waypoints(np.array(
                [[0, 0, 0, 1], [1, 0, 0, 1], [1, 1, 0, 1], [2, 1, 0, 1], [2, 1, 0, 0]]))
        if self.joystick.getRawButtonPressed(9):
            self.motion.disable()
            self.chassis.field_oriented = True

        if self.joystick.getRawButtonPressed(8):
            print("Heading sp set")
            self.chassis.set_heading_sp(self.bno055.getAngle() + math.pi)

        # this is where the joystick inputs get converted to numbers that are sent
        # to the chassis component. we rescale them using the rescale_js function,
        # in order to make their response exponential, and to set a dead zone -
        # which just means if it is under a certain value a 0 will be sent
        # TODO: Tune these constants forvwhatever robot they are on
        vx = -rescale_js(self.joystick.getY(), deadzone=0.05, exponential=1.2, rate=4)
        vy = -rescale_js(self.joystick.getX(), deadzone=0.05, exponential=1.2, rate=4)
        vz = -rescale_js(self.joystick.getZ(), deadzone=0.2, exponential=15.0, rate=self.spin_rate)
        self.chassis.set_inputs(vx, vy, vz)
=======

        # self.intake.intake_arm(self.xbox.getBButton())

        if self.xbox.getXButtonReleased():
            self.intake_automation.engage()
>>>>>>> d3c8d1f4aee24fe9a79a7a5a451e5bdb838412a7

        # self.intake.intake_arm(self.xbox.getBButton())

        if self.xbox.getXButtonReleased():
            self.intake_automation.engage()


if __name__ == '__main__':
    wpilib.run(Robot)
