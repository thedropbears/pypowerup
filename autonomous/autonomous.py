"""The autonomous controls for the robot. Vision code is not yet included"""
import math
import wpilib
import numpy as np
from magicbot.state_machine import AutonomousStateMachine, state
from components.vision import Vision
from automations.motion import ChassisMotion
from pyswervedrive.swervechassis import SwerveChassis
from utilities.bno055 import BNO055
from robot import Robot


class OverallBase(AutonomousStateMachine):
    """statemachine that is subclassed to all deal with all possible autonomous requirements."""

    vision: Vision
    bno055: BNO055
    motion: ChassisMotion
    chassis: SwerveChassis
    ds: wpilib.DriverStation

    cube_switch: wpilib.DigitalInput
    # should be the closest cube
    # cubes will be listed in size order along with thier rough size and angle

    def on_enable(self):
        self.game_data_message = self.ds.getGameSpecificMessage()
        self.fails = 0
        if len(self.game_data_message) == 3:
            self.fms_scale = self.game_data_message[1]  # L or R
            self.fms_switch = self.game_data_message[0]  # L or R
        else:
            # need defaults
            self.fms_scale = 'R'
            self.fms_switch = 'R'
        self.chassis.odometry_x = Robot.length / 2
        self.chassis.odometry_y = 0
        super().on_enable()

    @state(first=True)
    def go_to_scale(self):
        """The robot travels to the scale"""
        if self.fms_scale == 'L':
            # go to left scale
            pass
        if self.fms_scale == 'R':
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
        print('search for cube')
        if self.vision.largest_cube() is None:
            self.chassis.set_inputs(0, 0, 1)
        else:
            self.next_state("turn_and_go_to_cube")

    @state
    def turn_and_go_to_cube(self):
        """The robot rotates in the direction specified by the vision
        system while moving towards the cube. Combines two angles to find the absolute
        angle towards the cube"""
        angle = self.bno055.getAngle()
        vision_angle = self.vision.largest_cube()
        # print(vision_angle)
        if vision_angle is None:
            self.next_state_now("search_for_cube")  # tempoary for testing
            return
            print("========searching for cube========")
        if not self.cube_switch.get():
            self.next_state_now("go_to_scale")
            print('===========Going to scale===========')
            return
        else:
            print("no cube switch")
        absolute_cube_direction = angle + vision_angle
        new_heading = angle + 0.2 * vision_angle
        self.chassis.field_oriented = True
        self.chassis.set_velocity_heading(math.cos(absolute_cube_direction),
                                          math.sin(absolute_cube_direction),
                                          new_heading)
        # TODO: implement state transition
        # self.next_state("intake_cube")

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


class VisionTest(OverallBase):
    """To test the vision system"""
    DEFAULT = True
    MODE_NAME = 'Vision Test'

    @state
    def go_to_cube(self, initial_call):
        """The robot drives towards where the next cube should be"""
        print("I am going to the cube")
        if initial_call:
            angle = self.bno055.getAngle()
            self.motion.set_waypoints([[self.chassis.odometry_x, self.chassis.odometry_y, angle, 0],
                                       [2.5, 0, math.pi/2, 1.5],
                                       [2.5, 1, math.pi/2, 1.5]])
        if not self.motion.enabled:
            print("going to 'intake cube'")
            # self.next_state("search_for_cube")
            self.next_state_now("intake_cube")

    @state
    def intake_cube(self):
        self.chassis.field_oriented = True
        self.chassis.set_inputs(0, 1.5, 0)
        print("i am waiting to intake the cube")
        if True:  # not self.cube_switch.get():
            self.next_state_now("go_to_scale")
            print('===========Going to scale===========')
            return

    @state(first=True)
    def go_to_scale(self, initial_call):
        """The robot travels to the scale"""
        if initial_call:
            angle = self.bno055.getAngle()
            self.motion.set_waypoints([[self.chassis.odometry_x, self.chassis.odometry_y, angle, 0],
                                       [6, 0, 0, 0]])
        if not self.motion.enabled:
            print("================= At scale ======================")
            self.next_state("go_to_cube")


class SwitchAndScale(OverallBase):
    """A less general routine for the switch and scale strategy. Still requires subclassing"""

    def __init__(self):
        self.been_to_switch = False
        super().__init__()

    @state
    def go_to_switch(self):
        """Goes to the switch, when subclassed will go to the correct side regardless of start."""
        self.been_to_switch: True
        if self.fms_switch == 'L':
            # go to left switch
            pass
        if self.fms_switch == 'R':
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
        if self.fms_switch == 'L':
            self.next_state("go_to_switch")
        elif self.fms_scale == 'R':
            # go to right scale
            pass
        elif self.fms_scale == 'L':
            # go to left scale
            pass
        self.next_state("deposit_cube")


class RightSwitchAndScale(SwitchAndScale):
    """The switch and scale strategy when we start on the right"""
    MODE_NAME: 'Switch and scale - right start'
    start_position = -3  # change to right start position
    coordinates = [[[4.24, -2.0, 1.60, 1],
                    [5.5, -2.75, 2, 1],
                    [5.25, -1.91, 2, 1],
                    [7.5, -1.87, 0, 1]],  # SameSwitchSameScale
                   [[4.29, -1.91, 1.45, 1],
                    [5.5, -2.5, 1.8, 1],
                    [5.08, 1.90, 0, 1],
                    [7.07, 1.90, 0, 1]],  # SameSwitchCrossScale
                   [[7.5, -1.87, 0, 1],
                    [5.5, -2, 2, 1],
                    [6, 2.1, 4.71239, 1],
                    [4.2, 2.1, 4.71239, 1]],  # CrossSwitchSameScale
                   [[5, -3, 0, 1],
                    [6, 2, 0, 1],
                    [7.5, 1.8, 0, 1],
                    [5.5, 2, 3.92699, 1],
                    [5.25, 1.5, 3.14159, 1]]]  # CrossSwitchCrossScale

    @state(first=True)
    def go_to_scale(self):
        if self.fms_switch == 'R':
            self.next_state("go_to_switch")
        elif self.fms_scale == 'R':
            # go to right scale
            pass
        elif self.fms_scale == 'L':
            # go to left scale
            pass
        self.next_state("deposit_cube")


class LeftDoubleScale(OverallBase):
    """The double switch strategy when we start on the left"""
    MODE_NAME: 'Double scale - left start'
    coordinates = [[[8, 2.25, 1.5708, 1],
                    [6.10, 2.50, 2.35619, 1],
                    [5.25, 1.80, 2.35619, 1],
                    [7.50, 2, 0, 1]],  # coordinates for same scale
                   [[5, 3, 0, 1],
                    [6, -2, 0, 1],
                    [7.5, -1.8, 0, 1],
                    [5.5, -2, 3.92699, 1],
                    [7.5, -2, 0, 1]]]  # coordinates for cross scale

    @state(first=True)
    def go_to_scale(self):
        if self.fms_scale == 'R':
            self.coordinates = self.coordinates[1]
            self.motion.set_waypoints = np.array(self.coordinates[0])
            self.motion.set_waypoints = np.array(self.coordinates[1])
            self.motion.set_waypoints = np.array(self.coordinates[2])
            # go to right scale
        elif self.fms_scale == 'L':
            self.coordinates = self.coordinates[0]
            self.motion.set_waypoints = np.array(self.coordinates[0])
            # go to left scale
        self.next_state("deposit_cube")


class RightDoubleScale(OverallBase):
    """The double switch strategy when we start on the right"""
    MODE_NAME: 'Double scale - right start'
    coordinates = [[[8, -2.25, 1.5708, 1],
                    [6.10, -2.50, 2.35619, 1],
                    [5.25, -1.80, 2.35619, 1],
                    [7.50, -2, 0, 1]],  # coordinates for same scale
                   [[5, -3, 0, 1],
                    [6, 2, 0, 1],
                    [7.5, 1.8, 0, 1],
                    [5.5, 2, 3.92699, 1],
                    [7.5, 2, 0, 1]]]  # coordinates for cross scale

    @state(first=True)
    def go_to_scale(self):
        if self.fms_scale == 'R':
            self.coordinates = self.coordinates[0]
            self.motion.set_waypoints = np.array(self.coordinates[0])
            # go to right scale
        elif self.fms_scale == 'L':
            self.coordinates = self.coordinates[1]
            self.motion.set_waypoints = np.array(self.coordinates[0])
            self.motion.set_waypoints = np.array(self.coordinates[1])
            self.motion.set_waypoints = np.array(self.coordinates[2])
            # go to left scale
        self.next_state("deposit_cube")
