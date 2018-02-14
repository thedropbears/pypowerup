
"""The autonomous controls for the robot."""
import math

import wpilib
from magicbot.state_machine import AutonomousStateMachine, state

from automations.intake import IntakeAutomation
from automations.lifter import LifterAutomation
from automations.motion import ChassisMotion
from components.intake import Intake
from components.lifter import Lifter
from components.vision import Vision
from pyswervedrive.swervechassis import SwerveChassis
from robot import Robot
from utilities.navx import NavX


class OverallBase(AutonomousStateMachine):
    """statemachine designed to intelegently respond to possible situations in auto"""
    vision: Vision
    lifter: Lifter
    intake: Intake
    imu: NavX
    chassis: SwerveChassis
    ds: wpilib.DriverStation

    # automations
    motion: ChassisMotion
    intake_automation: IntakeAutomation
    lifter_automation: LifterAutomation

    START_Y_COORDINATE = 3

    CROSS_POINT_SPEED = 3

    # Coordinates of various objectives no the field
    # Default to those for LEFT HAND SIDE of the field
    SCALE_DEPOSIT = [7.6-Robot.length / 2, 1.8]
    SWITCH_DEPOSIT = [4.2, 1.9+Robot.length / 2]
    SWITCH_DEPOSIT_ORIENTATION = -math.pi/2
    PICKUP_WAYPOINT = [5.4, 1.8]
    CROSS_POINT = [5.4, 1.8]
    OPP_CROSS_POINT = [5.4, -1.8]
    CUBE_PICKUP_ORIENTATION = -math.pi
    CUBE_PICKUP_1 = [5, 1.7]
    CUBE_PICKUP_2 = [5, 1.4]

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

        self.cube_number = 0

        super().on_enable()

    @state
    def pick_up_cube(self, initial_call):
        """The robot rotates in the direction specified by the vision
        system while moving towards the cube. Combines two angles to find the absolute
        angle towards the cube"""
        vision_angle = self.vision.largest_cube()
        angle = self.imu.getAngle()
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

    @state
    def go_to_scale(self, initial_call):
        """Navigate to the scale. Raise the lift"""
        if initial_call:
            self.done_switch = True
            self.motion.set_waypoints([
                self.current_waypoint,
                self.SCALE_DEPOSIT+[0, 0]
                ])
        if not self.motion.enabled:
            self.next_state_now('deposit_scale')

    @state
    def deposit_scale(self, initial_call):
        """Deposit the cube on the scale."""
        if initial_call:
            self.chassis.set_inputs(0, 0, 0)
            self.cube_number += 1
            self.cube_inside = False
        # if not self.intake_automation.is_executing:
        if True:
            self.next_state_now('nav_to_cube')

    @property
    def current_waypoint(self):
        return [self.chassis.odometry_x, self.chassis.odometry_y,
                self.imu.getAngle(), self.chassis.speed]


class DoubleScaleBase(OverallBase):

    @state(first=True)
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
    def nav_to_cube(self, initial_call):
        """Navigate on Dead Reckoning to the correct cube."""
        if initial_call:
            if self.cube_number == 1:
                cube = self.CUBE_PICKUP_1
            elif self.cube_number >= 2:
                cube = self.CUBE_PICKUP_2
            self.motion.set_waypoints(([
                self.current_waypoint,
                self.PICKUP_WAYPOINT,
                cube
                ]))
        if not self.motion.enabled:
            self.next_state('go_to_scale')


class LeftDoubleScale(DoubleScaleBase):
    MODE_NAME = 'Left Double Scale'

    def on_enable(self):
        super().on_enable()
        self.start_side = 'L'
        self.current_side = self.start_side
        self.cube_inside = True

        if self.fms_scale == 'R':
            self.SCALE_DEPOSIT[1] *= -1
            self.CUBE_PICKUP_1[1] *= -1
            self.CUBE_PICKUP_2[1] *= -1
            self.CUBE_PICKUP_ORIENTATION *= -1
            self.PICKUP_WAYPOINT[1] *= -1


class RightDoubleScale(DoubleScaleBase):
    MODE_NAME = 'Right Double Scale'

    def on_enable(self):
        super().on_enable()
        self.start_side = 'R'
        self.current_side = self.start_side
        self.cube_inside = True

        self.CROSS_POINT, self.OPP_CROSS_POINT = self.OPP_CROSS_POINT, self.CROSS_POINT
        self.chassis.odometry_y = -self.START_Y_COORDINATE

        if self.fms_scale == 'R':
            self.SCALE_DEPOSIT[1] *= -1
            self.CUBE_PICKUP_1[1] *= -1
            self.CUBE_PICKUP_2[1] *= -1
            self.CUBE_PICKUP_ORIENTATION *= -1
            self.PICKUP_WAYPOINT[1] *= -1


class SwitchScaleBase(OverallBase):

    def decide_objective(self):
        if self.cube_inside:
            if self.current_side == self.fms_switch and not self.done_switch:
                print("Going to switch, current side %s switch side %s scale %s" % (self.current_side, self.fms_switch, self.fms_scale))
                self.next_state_now('go_to_switch')
            elif self.current_side == self.fms_scale:
                print("Going to scale, current side %s switch side %s scale %s" % (self.current_side, self.fms_switch, self.fms_scale))
                self.next_state_now('go_to_scale')
        else:
            self.next_state_now('nav_to_cube')

    @state(first=True)
    def cross_field(self, initial_call):
        """Cross the field."""
        if initial_call:
            if self.current_side in [self.fms_switch, self.fms_scale]:
                self.decide_objective()
                return
            else:
                self.motion.set_waypoints([
                    self.current_waypoint,
                    self.CROSS_POINT+[0, self.CROSS_POINT_SPEED],
                    self.OPP_CROSS_POINT+[0, self.CROSS_POINT_SPEED]
                    ])
        if not self.motion.enabled:
            self.current_side = 'R' if self.current_side == 'L' else 'L'
            self.decide_objective()
            return

    @state
    def go_to_switch(self, initial_call):
        """Navigate to the scale. Raise the lift"""
        if initial_call:
            self.done_switch = True
            self.motion.set_waypoints([
                self.current_waypoint,
                self.SWITCH_DEPOSIT+[self.SWITCH_DEPOSIT_ORIENTATION, 0]
                ])
        if not self.motion.enabled:
            self.next_state_now('deposit_switch')

    @state
    def nav_to_cube(self, initial_call):
        """Navigate on Dead Reckoning to the correct cube."""
        if initial_call:
            if self.cube_number == 1:
                cube = self.CUBE_PICKUP_1
            if self.cube_number >= 2:
                cube = self.CUBE_PICKUP_2
            self.motion.set_waypoints([
                self.current_waypoint,
                self.PICKUP_WAYPOINT+[self.CUBE_PICKUP_ORIENTATION, 3],
                cube+[self.CUBE_PICKUP_ORIENTATION, 3]
                ])
            self.intake_automation.engage(initial_state='intake_cube')
        # if self.intake.cube_inside():
        if not self.motion.enabled:
            self.cube_inside = True
            if self.done_switch:
                self.next_state_now('go_to_scale')
            else:
                self.next_state_now('cross_field')

    @state
    def deposit_switch(self, initial_call):
        """Deposit the cube on the switch."""
        if initial_call:
            self.chassis.set_inputs(0, 0, 0)
            self.intake_automation.engage(initial_state='deposit')
            self.cube_number += 1
            self.cube_inside = False
        # if not self.intake_automation.is_executing:
        if True:
            if self.current_side == self.fms_scale:
                self.next_state_now('nav_to_cube')
            else:
                self.next_state_now('cross_field')
            return


class LeftSwitchScale(SwitchScaleBase):
    MODE_NAME = 'Left Switch & Scale'

    def on_enable(self):
        super().on_enable()
        self.start_side = 'L'
        self.current_side = self.start_side
        self.done_switch = False
        self.cube_inside = True

        if self.fms_switch == 'R':
            self.SWITCH_DEPOSIT[1] *= -1
            self.SWITCH_DEPOSIT_ORIENTATION *= -1
        if self.fms_scale == 'R':
            self.CUBE_PICKUP_1[1] *= -1
            self.CUBE_PICKUP_2[1] *= -1
            self.CUBE_PICKUP_ORIENTATION *= -1
            self.PICKUP_WAYPOINT[1] *= -1
            self.SCALE_DEPOSIT[1] *= -1


class RightSwitchScale(SwitchScaleBase):
    DEFAULT = True
    MODE_NAME = 'Right Switch & Scale'

    def on_enable(self):
        super().on_enable()
        self.CROSS_POINT, self.OPP_CROSS_POINT = self.OPP_CROSS_POINT, self.CROSS_POINT
        self.chassis.odometry_y = -self.START_Y_COORDINATE

        print("FMS Switch %s" % self.fms_switch)
        print("FMS Scale %s" % self.fms_scale)
        if self.fms_switch == 'R':
            self.SWITCH_DEPOSIT[1] *= -1
            self.SWITCH_DEPOSIT_ORIENTATION *= -1
        if self.fms_scale == 'R':
            self.CUBE_PICKUP_1[1] *= -1
            self.CUBE_PICKUP_2[1] *= -1
            # self.CUBE_PICKUP_ORIENTATION *= -1
            self.PICKUP_WAYPOINT[1] *= -1
            self.SCALE_DEPOSIT[1] *= -1
        self.start_side = 'R'
        self.current_side = self.start_side
        self.done_switch = False
        self.cube_inside = True
