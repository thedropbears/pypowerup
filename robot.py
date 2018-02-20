#!/usr/bin/env python3

import ctre
import magicbot
import wpilib
from automations.intake import IntakeAutomation
from automations.lifter import LifterAutomation
from automations.motion import ChassisMotion
from components.intake import Intake
from components.lifter import Lifter
from components.vision import Vision
from components.range_finder import RangeFinder
from pyswervedrive.swervechassis import SwerveChassis
from pyswervedrive.swervemodule import SwerveModule
from utilities.navx import NavX
from utilities.functions import rescale_js
from robotpy_ext.common_drivers.distance_sensors import SharpIRGP2Y0A41SK0F
from robotpy_ext.misc.looptimer import LoopTimer
from networktables import NetworkTables
import math


class Robot(magicbot.MagicRobot):
    # Add magicbot components here using variable annotations.
    # Any components that directly actuate motors should be declared after
    # any higher-level components (automations) that depend on them.

    vision: Vision
    range_finder: RangeFinder

    # Automations
    motion: ChassisMotion
    intake_automation: IntakeAutomation
    lifter_automation: LifterAutomation

    # Actuators
    chassis: SwerveChassis
    intake: Intake
    lifter: Lifter

    module_drive_free_speed: float = 7800.  # encoder ticks / 100 ms
    length: float = 0.88

    def createObjects(self):
        """Create non-components here."""

        self.module_a = SwerveModule(  # top left module
            "a", steer_talon=ctre.TalonSRX(48), drive_talon=ctre.TalonSRX(49),
            x_pos=0.25, y_pos=0.31,
            drive_free_speed=Robot.module_drive_free_speed)
        self.module_b = SwerveModule(  # bottom left modulet
            "b", steer_talon=ctre.TalonSRX(46), drive_talon=ctre.TalonSRX(47),
            x_pos=-0.25, y_pos=0.31,
            drive_free_speed=Robot.module_drive_free_speed)
        self.module_c = SwerveModule(  # bottom right modulet
            "c", steer_talon=ctre.TalonSRX(44), drive_talon=ctre.TalonSRX(45),
            x_pos=-0.25, y_pos=-0.31,
            drive_free_speed=Robot.module_drive_free_speed)
        self.module_d = SwerveModule(  # top right modulet
            "d", steer_talon=ctre.TalonSRX(42), drive_talon=ctre.TalonSRX(43),
            x_pos=0.25, y_pos=-0.31,
            drive_free_speed=Robot.module_drive_free_speed)

        self.intake_left_motor = ctre.TalonSRX(14)
        self.intake_right_motor = ctre.TalonSRX(2)
        self.clamp_arm = wpilib.Solenoid(2)
        self.intake_kicker = wpilib.Solenoid(3)
        self.left_extension = wpilib.Solenoid(6)
        self.right_extension = wpilib.DoubleSolenoid(forwardChannel=5, reverseChannel=4)
        self.left_infrared = SharpIRGP2Y0A41SK0F(5)
        self.right_infrared = SharpIRGP2Y0A41SK0F(6)
        self.compressor = wpilib.Compressor()
        self.lifter_motor = ctre.TalonSRX(3)
        self.centre_switch = wpilib.DigitalInput(1)
        self.top_switch = wpilib.DigitalInput(2)

        # create the imu object
        self.imu = NavX()
        # wpilib.SmartDashboard.putData('gyro', self.imu.ahrs)

        # boilerplate setup for the joystick
        self.joystick = wpilib.Joystick(0)
        self.gamepad = wpilib.XboxController(1)

        self.spin_rate = 2

        self.sd = NetworkTables.getTable("SmartDashboard")

        self.range_finder_counter = wpilib.Counter(4, mode=wpilib.Counter.Mode.kPulseLength)

    def teleopInit(self):
        '''Called when teleop starts; optional'''
        self.motion.enabled = False
        self.chassis.set_inputs(0, 0, 0)
        self.loop_timer = LoopTimer(self.logger)

    def teleopPeriodic(self):
        """
        Process inputs from the driver station here.

        This is run each iteration of the control loop before magicbot components are executed.
        """

        if self.joystick.getRawButtonPressed(12):
            self.intake.clamp_on = not self.intake.clamp_on

        if self.joystick.getRawButtonPressed(11):
            self.intake.push_on = not self.intake.push_on

        if self.joystick.getTrigger():
            self.intake_automation.engage(initial_state="intake_cube")

        if self.joystick.getRawButtonPressed(4) or self.gamepad.getTriggerAxis(wpilib.interfaces.GenericHID.Hand.kRight) > 0.5:
            self.intake_automation.engage(initial_state="eject_cube")

        if self.joystick.getRawButtonPressed(2) or self.gamepad.getStartButtonPressed():
            self.intake_automation.engage(initial_state="stop", force=True)

        if self.joystick.getRawButtonPressed(3) or self.gamepad.getTriggerAxis(wpilib.interfaces.GenericHID.Hand.kLeft) > 0.5:
            self.intake_automation.engage(initial_state="exchange")

        if self.gamepad.getAButtonPressed():
            self.lifter_automation.engage(initial_state="move_upper_scale", force=True)

        if self.gamepad.getBButtonPressed():
            self.lifter_automation.engage(initial_state="move_balanced_scale", force=True)

        if self.gamepad.getYButtonPressed():
            self.lifter_automation.engage(initial_state="move_lower_scale", force=True)

        if self.gamepad.getXButtonPressed():
            self.lifter_automation.engage(initial_state="move_switch", force=True)

        if self.gamepad.getBackButtonPressed():
            self.lifter_automation.engage(initial_state="reset", force=True)

        if self.joystick.getRawButtonPressed(10):
            self.imu.resetHeading()
            self.chassis.set_heading_sp_current()

        if self.joystick.getRawButtonPressed(8):
            print("Heading sp set")
            self.chassis.set_heading_sp(self.imu.getAngle() + math.pi)

        # this is where the joystick inputs get converted to numbers that are sent
        # to the chassis component. we rescale them using the rescale_js function,
        # in order to make their response exponential, and to set a dead zone -
        # which just means if it is under a certain value a 0 will be sent
        # TODO: Tune these constants for whatever robot they are on
        vx = -rescale_js(self.joystick.getY(), deadzone=0.1, exponential=1.5, rate=4)
        vy = -rescale_js(self.joystick.getX(), deadzone=0.1, exponential=1.5, rate=4)
        vz = -rescale_js(self.joystick.getZ(), deadzone=0.4, exponential=10.0, rate=self.spin_rate)
        self.chassis.set_inputs(vx, vy, vz)

        self.loop_timer.measure()

    def testPeriodic(self):
        if self.gamepad.getStartButtonPressed():
            self.module_a.store_steer_offsets()
            self.module_b.store_steer_offsets()
            self.module_c.store_steer_offsets()
            self.module_d.store_steer_offsets()

    def robotPeriodic(self):
        # super().robotPeriodic()
        if self.lifter.set_pos is not None:
            self.sd.putNumber("lift/set_pos", self.lifter.set_pos)
        # self.sd.putNumber("lift/current", self.lifter.motor.getOutputCurrent())


if __name__ == '__main__':
    wpilib.run(Robot)
