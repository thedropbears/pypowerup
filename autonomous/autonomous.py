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
    """statemachine designed to intelegently respond to possible situations in auto"""

    vision: Vision
    bno055: BNO055
    motion: ChassisMotion
    chassis: SwerveChassis
    ds: wpilib.DriverStation

    cube_switch: wpilib.DigitalInput
    # should be the closest cube
    # cubes will be listed in size order along with thier rough size and angle


    def on_enable(self):
        self.game_data_message = self.ds.getGameSpecificMessage()  # TODO test this
        self.toggle_searching_closest_objective=True
        #make y +ve or -ve depending on where we start
        self.navigation_point=[5.6, 2.4, 0, 1]
        self.strategy = 'double_scale'  # this will be set by the dashboard
        self.start_side = 'R' # set by the dashboard
        self.target_objective = 'R_scale'   

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

    @state(first=true)
    def setup(self):
        if switch_first==true:
            self.next_state("go_to_switch")
            #change switch postion one off
        else:
            self.next_state("navigation_point")

    def invert_co_ordinates(co_ordinate):
        for i in co_ordinate:
            co_ordinate[1] *= -1
        return co_ordinate

    def close_to_objective(objective):
        if self.start_side == self.fms_switch and objective == 'switch':
            return True
        else:
            return False
        if self.start_side == self.fms_scale and objective == 'scale':
            return True
        else:
            return False

    def target_objective():
        """A dictionary function that gives an output on the order of objectives based on inputs.
        The inputs are designated on the left in a tuple in the order of robot start side, strategy, switch side
        scale side. The outputs are given on the right in the order which the robot needs to go to"""
        objective_calculation = {
            # Double scale with right start
            ('R','double_scale','L','L'): ('cross_l_scale', 'same_l_scale'),
            ('R','double_scale','R','L'): ('cross_l_scale', 'same_l_scale'),
            ('R','double_scale','R','R'): ('same_r_scale', 'same_r_scale'),
            ('R','double_scale','L','R'): ('same_r_scale', 'same_r_scale'),
            # Double scale with left start
            ('L','double_scale','L','L'): ('same_l_scale', 'same_l_scale'),
            ('L','double_scale','R','L'): ('same_l_scale', 'same_l_scale'),
            ('L','double_scale','R','R'): ('cross_r_scale', 'same_r_scale'),
            ('L','double_scale','L','R'): ('cross_r_scale', 'same_r_scale'),
            # Switch and scale with right start
            ('R','switch_and_scale','L','R'): ('cross_l_scale', 'same_r_scale'),
            ('R','switch_and_scale','L','L'): ('cross_l_scale', 'same_l_switch'),
            ('R','switch_and_scale','R','L'): ('same_r_switch', 'cross_l_scale'),
            ('R','switch_and_scale','R','R'): ('same_r_switch', 'same_r_scale'),
            # Switch and scale with left start
            ('L','switch_and_scale','R','L'): ('same_l_scale', 'cross_r_switch'),
            ('L','switch_and_scale','R','R'): ('cross_r_scale', 'same_r_switch'),
            ('L','switch_and_scale','L','R'): ('same_l_switch', 'cross_r_scale'),
            ('L','switch_and_scale','L','L'): ('same_l_switch', 'same_l_scale')
        }
        return objective_calculation[(self.start_side, self.strategy, self.fms_switch, self.fms_scale)]

    @state(first=True)
    def navigation_point(self, initial_call):
        if initial_call:
            #seraching for objective
            if toggle_searching_for_objective:
                if close_to_objective(self.target_objective):
                    # at correct nav point
                    self.motion.set_waypoints([[self.chassis.odometry_x, self.chassis.odometry_y, angle, 0],self.navigation_point])
                else:
                    # go to other navigation point
                    self.navpoint = invert_co_ordinates(self.navpoint)
                    self.motion.set_waypoints([[self.chassis.odometry_x, self.chassis.odometry_y, angle, 0],self.navigation_point])


                if self.target_objective == 'switch':
                    self.next_state('go_to_switch')
                else:
                    self.next_state('go_to_scale')
            else:
                #serach for cube , nav  point close to us
                self.motion.set_waypoints([[self.chassis.odometry_x, self.chassis.odometry_y, angle, 0],self.navigation_point])
                self.next_state("go_to_cube")
            #self.toggle_searching_closest_objective=not(toggle_searching_closest_objective)
            
    @state
    def go_to_cube(self, initial_call):
        """The robot drives towards where the next cube should be"""
        if initial_call:
            angle = self.bno055.getAngle()
            self.motion.set_waypoints([[self.chassis.odometry_x, self.chassis.odometry_y, angle, 0]])
        if not self.motion.enabled:

            self.next_state_now("deposit_cube") 

    @state
    def deposit_cube(self):             
        # This is for depositing the cube to either the Scale or The switch 
        self.next_state("navigation_point")

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
        absolute_cube_direction = angle + vision_angle
        new_heading = angle + 0.2 * vision_angle
        self.chassis.field_oriented = True
        self.chassis.set_velocity_heading(math.cos(absolute_cube_direction),
                                          math.sin(absolute_cube_direction),
                                          new_heading)
        self.next_state("intake_cube")

    @state
    def search_for_cube(self):
        """The robot attemppts to find a cube within the frame of the camera"""
        if self.vision.largest_cube() is None:
            self.chassis.set_inputs(0, 0, 1)
        else:
            self.next_state("turn_and_go_to_cube")

    @state
    def go_to_switch(self):
        """The robot travels to the switch"""
        if self.fms_switch == 'L':
            # go to left switch
            pass
        if self.fms_switch == 'R':
            # go to right switch
            pass
        self.next_state("deposit_cube")

    @state
    def go_to_scale(self):
        """The robot travels to the scale"""
        if self.fms_scale == 'L':
            # go to left scale
            pass
        if self.fms_scale == 'R':
            # go to right scale
            pass
        self.next_state("deposit_cube")

class VisionTest(OverallBase):
    """To test the vision system"""
    DEFAULT = True
    MODE_NAME = 'Vision Test'

    @state
    def go_to_cube(self, initial_call):
        """The robot drives towards where the next cube should be"""
        if initial_call:
            angle = self.bno055.getAngle()
            self.motion.set_waypoints([[self.chassis.odometry_x, self.chassis.odometry_y, angle, 0],
                                       [2.5, 0, math.pi/2, 1],
                                       [2.5, 1, math.pi/2, 0]])
        if not self.motion.enabled:
            # print("going to 'search_for_cube'")
            # self.next_state("search_for_cube")
            self.next_state_now("deposit_cube")

    @state
    def intake_cube(self):
        self.chassis.set_inputs(0, 1.5, 0)
        self.chassis.field_oriented = True
        if not self.cube_switch.get():
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

