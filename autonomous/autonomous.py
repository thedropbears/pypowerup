
"""The autonomous controls for the robot."""
import math

import wpilib
from magicbot.state_machine import AutonomousStateMachine, state

from automations.intake import IntakeAutomation
from automations.lifter import LifterAutomation
from automations.motion import ChassisMotion
from components.lifter import Lifter
from components.vision import Vision
from pyswervedrive.swervechassis import SwerveChassis
from robot import Robot
from utilities.bno055 import BNO055


class OverallBase(AutonomousStateMachine):
    """statemachine designed to intelegently respond to possible situations in auto"""
    vision: Vision
    lifter: Lifter
    bno055: BNO055
    chassis: SwerveChassis
    ds: wpilib.DriverStation

    # automations
    motion: ChassisMotion
    intake_automation: IntakeAutomation
    lifter_automation: LifterAutomation

    cube_switch: wpilib.DigitalInput  # the switch used to confirm cube capture during early testing

    START_Y_COORDINATE = 3

    CROSS_POINT_SPEED = 3

    # Coordinates of various objectives no the field
    # Default to those for LEFT HAND SIDE of the field
    SCALE_DEPOSIT = [7.6-Robot.length / 2, 1.8]
    SWITCH_DEPOSIT = [4.2, 1.9+Robot.length / 2]
    SWITCH_DEPOSIT_ORIENTATION = math.pi/2
    CROSS_POINT = [5.4, 1.8]
    OPP_CROSS_POINT = [5.4, -1.8]
    CUBE_PICKUP_ORIENTATION = 0
    CUBE_PICKUP_1 = [0, 0]
    CUBE_PICKUP_2 = [0, 0]

    def on_enable(self):
        # self.lifter.reset() do we need this?
        self.game_data_message = self.ds.getGameSpecificMessage()

        if len(self.game_data_message) == 3:
            self.fms_scale = self.game_data_message[1]  # L or R
            self.fms_switch = self.game_data_message[0]  # L or R
        else:
            # need defaults
            self.fms_scale = 'R'
            self.fms_switch = 'R'

        self.chassis.odometry_x = Robot.length / 2

        super().on_enable()

    @state
    def lifting(self, initial_call):
        """The robot lifts then releases its cube into either the scale or switch.
        Makes use of the external lifting statemachine"""
        if initial_call:
            self.picking_up_cube = True
            # toggles the navpoint to cube pickup mode
            self.navigation_point[3] = 0
            if self.chassis.odometry_y < 0:
                self.navigation_point[2] = 2.5 * math.pi / 4
            else:  # changes the facing of the navpoint based on which side the
                # robot is on TODO test this!
                self.navigation_point[2] = - 2.5 * math.pi / 4
            self.intake_automation.engage(initial_state="deposit")
        # TODO replace lifter statemachine
        # Release cube
        # if not self.intake_automation.is_executing:
        self.next_state("depositing")

    @state
    def depositing(self, state_tm, initial_call):
        if initial_call:
            state_tm = 0
        self.chassis.set_inputs(-1, 0, 0)
        if state_tm >= 0.25:
            self.chassis.set_inputs(0, 0, 0)
            self.next_state("navigating")

    @state
    def pick_up_cube(self, initial_call):
        """The robot rotates in the direction specified by the vision
        system while moving towards the cube. Combines two angles to find the absolute
        angle towards the cube"""
        vision_angle = self.vision.largest_cube()
        angle = self.bno055.getAngle()
        if initial_call:
            # print(vision_angle)
            self.intake_automation.engage()
            self.navigation_point[3] = 3
        if vision_angle is None:
            self.next_state("search_for_cube")
            print("========searching for cube========")
            return
        absolute_cube_direction = angle + vision_angle
        new_heading = angle + 0.2 * vision_angle
        self.chassis.field_oriented = True
        self.chassis.set_velocity_heading(math.cos(absolute_cube_direction),
                                          math.sin(absolute_cube_direction),
                                          new_heading)
        if not self.motion.enabled:
            self.next_state("next_objective")


class DoubleScaleBase(OverallBase):

    @state
    def cross_field(self, initial_call):
        """Cross the field."""
        if initial_call:
            if self.start_side == self.fms_scale:
                self.next_state_now("go_to_scale")
                return
            else:
                self.motion.set_waypoints([
                    self.current_waypoint,
                    self.CROSS_POINT+[0, self.CROSS_POINT_SPEED],
                    self.OPP_CROSS_POINT+[0, self.CROSS_POINT_SPEED]
                    ])
        if not self.motion.enabled:
            self.next_state_now("go_to_scale")

    @state
    def go_to_scale(self, initial_call):
        """Navigate to the scale. Raise the lift"""
        pass

    @state
    def nav_to_cube(self, initial_call):
        """Navigate on Dead Reckoning to the correct cube."""
        pass

    @state
    def next_objective(self, initial_call):
        """Go directly to the go_to_scale state. Called by the super class."""
        pass

    @state
    def deposit(self, initial_call):
        """Deposit the cube"""
        pass


class LeftDoubleScale(DoubleScaleBase):
    DEFAULT = True
    MODE_NAME = 'Left Double Scale'

    def on_enable(self):
        self.start_side = 'L'


class RightDoubleScale():
    MODE_NAME = 'Right Double Scale'

    def on_enable(self):
        super().on_enable()
        self.CROSS_POINT, self.OPP_CROSS_POINT = self.CROSS_POINT, self.OPP_CROSS_POINT
        self.chassis.odometry_y = -self.START_Y_COORDINATE

        self.SCALE_DEPOSIT[1] *= -1
        self.CUBE_PICKUP_1[1] *= -1
        self.CUBE_PICKUP_2[1] *= -1
        self.CUBE_PICKUP_ORIENTATION *= -1
        self.start_side = 'R'
