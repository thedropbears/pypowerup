"""The autonomous controls for the robot. Vision code is not yet included"""
import math
import wpilib
import vision
import numpy as np
from magicbot.state_machine import AutonomousStateMachine, state
from automations.motion import ChassisMotion
from pyswervedrive.swervechassis import SwerveChassis
from utilities.bno055 import BNO055


class OverallBase(AutonomousStateMachine):
    """statemachine that is subclassed to all deal with all possible autonomous requirements."""

    GameData = wpilib.DriverStation.getInstance()
    GameDataMessage = GameData.getGameSpecificMessage()
    # Takes the sides of the scale from the field management system
    bno055: BNO055
    FMS_scale = GameDataMessage[1]  # L or R
    FMS_switch = GameDataMessage[0]  # L or R
    waypoints: ChassisMotion
    chassis: SwerveChassis
    vision_angle = [tuple(1)]
    target_cube = vision_angle[0][0]
    # the 0th element of the tuple at index 0.
    # should be the closest cube
    # cubes will be listed in size order along with thier rough size and angle
    fails = 0

    @state(first=True)
    def go_to_scale(self):
        """The robot travels to the scale"""
        if self.FMS_scale == 'L':
            # go to left scale
            pass
        if self.FMS_scale == 'R':
            # go to right scale
            pass
        self.next_state("deposit_cube")

    @state
    def deposit_cube(self):
        """The robot releases its cube into either the scale or switch"""
        # Release cube
        self.next_state("go_to_cube")

    @state
    def go_to_cube(self):  # this will need to be overridden in the subclasses
        """The robot drives towards where the next cube should be"""
        # go to closest_cube
        if 1 == 1:  # Gets to cube position (odometry)
            self.next_state("search_for_cube")

    @state
    def search_for_cube(self):
        """The robot attemppts to find a cube within the frame of the camera"""
        if self.vision_angle is not None:  # cube found
            self.next_state("turn_and_go_to_cube")
        elif self.fails >= 5:  # multiple failures
            self.next_state("dead_reckon") 
        else:
            self.fails += 1

    @state
    def turn_and_go_to_cube(self):
        """The robot rotates in the direction specified by the vision
        system while moving towards the cube"""
        angle = self.bno055.getAngle()
        absolute_cube_direction = angle + self.vision_angle
        self.chassis.set_inputs(math.cos(absolute_cube_direction),
                                math.sin(absolute_cube_direction),
                                (self.vision_angle/57.2958))
        self.next_state("intake_cube")

    @state
    def intake_cube(self):
        """Attempts to intake the cube"""
        # Run intake
        if 1 == 1:  # cube inside intake system (current spikes)
            self.next_state("go_to_scale")
        elif 2 == 2:  # After completing the intake cycle there is no cube
            self.next_state("search_for_cube")

    @state
    def dead_reckon(self):
        """The robot tries to find the cube without the assistance of vision.
        likely to fail used as a last resort if vision has failed multiple times."""
        # go to cube and run intake


class SwitchAndScale(OverallBase):
    """A less general routine for the switch and scale strategy. Still requires subclassing"""

    def __init__(self):
        self.been_to_switch = False
        super().__init__()

    @state
    def go_to_switch(self):
        """Goes to the switch, when subclassed will go to the correct side regardless of start."""
        self.been_to_switch: True
        if self.FMS_switch == 'L':
            # go to left switch
            pass
        if self.FMS_switch == 'R':
            # go to right switch
            pass
        self.next_state("deposit_cube")

    @state
    def intake_cube(self):
        """Attempts to intake the cube"""
        # Run intake
        if self.been_to_switch:  # cube inside intake system (current spikes)
            self.next_state("go_to_scale")
        elif self.been_to_switch is False:  # switch needs a cube and pickup was successful
            self.next_state("go_to_switch")
        if 1 == 1:  # After completing the intake cycle there is no cube
            self.next_state("search_for_cube")


class LeftSwitchAndScale(SwitchAndScale):
    """The switch and scale strategy when we start on the left"""
    MODE_NAME: 'Switch and scale - left start'
    start_position = 3  # change to left start position

    @state(first=True)
    def go_to_scale(self):
        if self.FMS_switch == 'L':
            self.next_state("go_to_switch")
        elif self.FMS_scale == 'R':
            # go to right scale
            pass
        elif self.FMS_scale == 'L':
            # go to left scale
            pass
        self.next_state("deposit_cube")


class RightSwitchAndScale(SwitchAndScale):
    """The switch and scale strategy when we start on the right"""
    MODE_NAME: 'Switch and scale - right start'
    start_position = -3  # change to right start position

    @state(first=True)
    def go_to_scale(self):
        if self.FMS_switch == 'R':
            self.next_state("go_to_switch")
        elif self.FMS_scale == 'R':
            # go to right scale
            pass
        elif self.FMS_scale == 'L':
            # go to left scale
            pass
        self.next_state("deposit_cube")


class LeftDoubleScale(OverallBase):
    """The double switch strategy when we start on the left"""
    MODE_NAME: 'Double scale - left start'
    coordinates = [[8, 2.25, 1.5708, 1],
                   [6.10, 2.50, 2.35619, 1],
                   [5.25, 1.80, 2.35619, 1],
                   [7.50, 2, 0, 1]]

    @state(first=True)
    def go_to_scale(self):
        if self.FMS_scale == 'R':
            pass
            # go to right scale
        elif self.FMS_scale == 'L':
            self.waypoints.set_waypoints = np.array(self.coordinates[0])            
            # go to left scale
        self.next_state("deposit_cube")


class RightDoubleScale(OverallBase):
    """The double switch strategy when we start on the right"""
    MODE_NAME: 'Double scale - right start'
    coordinates = [[8, -2.25, 1.5708, 1],
                   [6.10, -2.50, 2.35619, 1],
                   [5.25, -1.80, 2.35619, 1],
                   [7.50, -2, 0, 1]]
    @state(first=True)
    def go_to_scale(self):
        if self.FMS_scale == 'R':
            self.waypoints.set_waypoints = np.array(self.coordinates[0])
            # go to right scale
        elif self.FMS_scale == 'L':
            # go to left scale
            pass
        self.next_state("deposit_cube")
