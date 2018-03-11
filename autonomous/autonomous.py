
"""The autonomous controls for the robot."""
import math

import numpy as np
import wpilib
from magicbot.state_machine import AutonomousStateMachine, state

from automations.intake import IntakeAutomation
from automations.lifter import LifterAutomation
from automations.motion import ChassisMotion
from components.lifter import Lifter
from components.intake import Intake
from components.vision import Vision
from pyswervedrive.swervechassis import SwerveChassis
from utilities.imu import IMU

from wpilib import SmartDashboard


class OverallBase(AutonomousStateMachine):
    """statemachine designed to intelegently respond to possible situations in auto"""
    vision: Vision
    lifter: Lifter
    intake: Intake
    imu: IMU
    chassis: SwerveChassis
    ds: wpilib.DriverStation

    robot_width = 1
    robot_length = 0.88

    # automations
    motion: ChassisMotion
    intake_automation: IntakeAutomation
    lifter_automation: LifterAutomation

    START_Y_COORDINATE = 3.4 - robot_width / 2

    # Coordinates of various objectives no the field
    # Default to those for LEFT HAND SIDE of the field
    # TODO: determine how far forward/back of this we want to go
    SCALE_DEPOSIT = [7.8, 2]
    SCALE_DEPOSIT_WAYPOINT = [6, 1.9]
    CUBE_PICKUP_1 = [5+robot_length / 2, 1.75]
    CUBE_PICKUP_2 = [5+robot_length / 2, 1.03]
    SWITCH_DEPOSIT = [5+robot_length / 2, 1.2]
    SCALE_INIT_WAYPOINT = [5.5, 3]

    SWITCH_DEPOSIT_ORIENTATION = -math.pi

    CUBE_PICKUP_ORIENTATION = -math.pi

    PICKUP_WAYPOINT_X = 6
    CROSS_POINT = [6, START_Y_COORDINATE]
    OPP_CROSS_POINT = [6, -START_Y_COORDINATE]
    DRIVE_BY_SWITCH_POINT = [3.6, 2+robot_length / 2]
    """
    START_Y_COORDINATE = 1
    SCALE_DEPOSIT = [6-robot_length / 2, 1]
    # CUBE_PICKUP_1 = [3+robot_length / 2, 0.5]
    # CUBE_PICKUP_2 = [3+robot_length / 2, -0.5]
    # CUBE_PICKUP_1 = [3+1, 0.5]
    CUBE_PICKUP_1 = [3+1, 0.5]
    CUBE_PICKUP_2 = [3+1, -0.5]
    SWITCH_DEPOSIT = [3+robot_length / 2, 0]

    SWITCH_DEPOSIT_ORIENTATION = -math.pi

    CUBE_PICKUP_ORIENTATION = -math.pi

    PICKUP_WAYPOINT_X = 5
    SWITCH_TO_CUBE_POINT = [PICKUP_WAYPOINT_X, 0.8]

    DRIVE_BY_SWITCH_POINT = [2, 0.5+robot_length/2]
    CROSS_POINT = [4.5, 1]
    OPP_CROSS_POINT = [4.5, -1]
    """

    SWITCH_TO_CUBE_POINT = [PICKUP_WAYPOINT_X, 1.8]

    DRIVE_BY_ORIENTATION = -math.pi / 2
    DRIVE_BY_SPEED = 1

    # CUBE_RUN_MOTION = (1.5, 1.5, 1.5)
    CUBE_RUN_MOTION = (1, 1, 1)

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

        self.chassis.odometry_x = self.robot_length / 2

        self.cube_number = 0

        self.intake.clamp(True)
        self.intake.push(False)

        self.slow_scale = False

        super().on_enable()

    @state
    def nav_to_cube(self, initial_call):
        """Navigate on Dead Reckoning to the correct cube."""
        if initial_call:
            if self.cube_number == 1:
                self.cube = np.array(self.CUBE_PICKUP_1)
            elif self.cube_number >= 2:
                self.cube = np.array(self.CUBE_PICKUP_2)
            pickup_waypoint = [self.PICKUP_WAYPOINT_X, self.cube[1]]
            if self.chassis.odometry_x > 6.0:
                self.motion.set_trajectory([
                    self.current_waypoint,
                    self.SCALE_DEPOSIT_WAYPOINT,
                    pickup_waypoint,
                    self.cube], end_heading=self.CUBE_PICKUP_ORIENTATION, end_speed=1,
                    # DO NOT SMOOTH WAYPONTS HERE (it breaks things)
                    smooth=False, motion_params=self.CUBE_RUN_MOTION)
            else:
                self.motion.set_trajectory([
                    self.current_waypoint,
                    pickup_waypoint,
                    self.cube], end_heading=self.CUBE_PICKUP_ORIENTATION, end_speed=1,
                    # DO NOT SMOOTH WAYPONTS HERE (it breaks things)
                    smooth=False, motion_params=self.CUBE_RUN_MOTION)
            self.intake_automation.engage(initial_state='intake_cube', force=True)
        # if self.intake.is_cube_contained():
        #     print("Cube contained in nav to cube")
        #     self.next_objective()
        #     return
        if not self.motion.trajectory_executing:
            # TODO: navigate via vision once we get odometry-based movement
            # working well
            self.next_state_now('pick_up_cube')

    @state
    def pick_up_cube(self, initial_call):
        """The robot rotates in the direction specified by the vision
        system while moving towards the cube. Combines two angles to find the absolute
        angle towards the cube"""

        if initial_call:
            self.intake_automation.engage(initial_state='intake_cube')
            self.pickup_start_pos = self.chassis.position

        if self.intake.is_cube_contained():
            print("Intaken cube, going to next objective")
            self.next_state_now('stop')
            return

        vision_angle = self.vision.largest_cube()
        heading = self.imu.getAngle()
        if vision_angle is None:
            print("Don't see cube in vision")
            return
        alignment_direction = heading + vision_angle * 1.5
        self.chassis.field_oriented = True

        # speed controller
        segment = self.cube - self.pickup_start_pos
        displacement = self.cube - self.chassis.position.reshape(2)
        total_dist = np.linalg.norm(segment)
        dist_along_segment = total_dist - np.linalg.norm(displacement)
        if dist_along_segment < 0:
            dist_along_segment = 0
        # slowly ramp down the speed as we approach the cube

        speed = 1
        SmartDashboard.putNumber('cube_pickup_speed', speed)
        SmartDashboard.putNumber('vision_angle', vision_angle)
        SmartDashboard.putNumber('vision_alignment_heading', alignment_direction)

        vx = speed*math.cos(alignment_direction)
        vy = speed*math.sin(alignment_direction)
        self.chassis.set_velocity_heading(vx, vy, math.pi)

    @state
    def stop(self, initial_call, state_tm):
        if initial_call:
            self.chassis.set_inputs(0, 0, 0)
        if self.chassis.speed < 0.1 or state_tm > 1:
            self.next_objective()

    @state
    def go_to_scale(self, initial_call, state_tm):
        """Navigate to the scale. Raise the lift"""
        if initial_call:
            self.done_switch = True
            self.slow_engage = False
            if self.chassis.odometry_x < 1:
                self.slow_engage = True
                self.motion.set_trajectory([
                    self.current_waypoint,
                    self.SCALE_INIT_WAYPOINT,
                    self.SCALE_DEPOSIT_WAYPOINT,
                    self.SCALE_DEPOSIT
                    ], end_heading=0)
            elif self.slow_scale:
                self.motion.set_trajectory([
                    self.current_waypoint,
                    self.SCALE_DEPOSIT_WAYPOINT,
                    self.SCALE_DEPOSIT
                    ], end_heading=0, motion_params=self.CUBE_RUN_MOTION)
            else:
                self.motion.set_trajectory([
                    self.current_waypoint,
                    self.SCALE_DEPOSIT_WAYPOINT,
                    self.SCALE_DEPOSIT
                    ], end_heading=0)
            if not self.slow_engage:
                self.lifter_automation.engage(initial_state='move_upper_scale')
            print(f'Lifter slow engage {self.slow_engage}')
        if self.motion.linear_position > 5 and self.slow_engage:
            self.lifter_automation.engage(initial_state='move_upper_scale')
        # if not self.motion.trajectory_executing and state_tm > 4:
        if not self.motion.trajectory_executing:
            self.next_state_now('deposit_scale')

    @state
    def cross_to_scale(self, initial_call, state_tm):
        if initial_call:
            self.motion.set_trajectory([
                self.current_waypoint,
                self.CROSS_POINT,
                self.OPP_CROSS_POINT,
                self.SCALE_DEPOSIT
                ], end_heading=0)
        if self.motion.linear_position > 8:
            if not self.lifter_automation.is_executing:
                print('Lifter engage')
            self.lifter_automation.engage(initial_state='move_upper_scale')
        if not self.motion.trajectory_executing:
            self.next_state_now("deposit_scale")

    @state
    def deposit_scale(self, initial_call, state_tm):
        """Deposit the cube on the scale."""
        if initial_call:
            self.chassis.set_inputs(0, 0, 0)
            self.cube_number += 1
            self.cube_inside = False
            self.intake_automation.engage(initial_state='eject_cube', force=True)
            print("Ejecting Cube")
        if not self.intake_automation.is_executing and state_tm > 0.04:
            print(f'Deposited {state_tm}')
            self.lifter_automation.engage(initial_state='reset_wait', force=True)
            self.next_state_now('nav_to_cube')

    @property
    def current_waypoint(self):
        return [self.chassis.odometry_x, self.chassis.odometry_y]


class DoubleScaleBase(OverallBase):

    @state(first=True)
    def cross_field(self, initial_call, state_tm):
        """Cross the field."""
        if self.start_side == self.fms_scale:
            self.next_state_now("go_to_scale")
            return
        else:
            self.next_state_now('cross_to_scale')
            return

    def next_objective(self):
        self.slow_scale = True
        self.next_state_now('go_to_scale')


class LeftDoubleScale(DoubleScaleBase):
    MODE_NAME = 'Left Double Scale'

    def on_enable(self):
        super().on_enable()
        self.start_side = 'L'
        self.current_side = self.start_side
        self.cube_inside = True

        self.chassis.odometry_y = self.START_Y_COORDINATE

        if self.fms_scale == 'R':
            print("FMS Scale Right")
            self.SCALE_DEPOSIT[1] *= -1
            self.SCALE_DEPOSIT_WAYPOINT[1] *= -1
            self.CUBE_PICKUP_1[1] *= -1
            self.CUBE_PICKUP_2[1] *= -1
            self.CUBE_PICKUP_ORIENTATION *= -1
            self.SCALE_INIT_WAYPOINT[1] *= -1


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
            print("FMS Scale Right")
            self.SCALE_DEPOSIT[1] *= -1
            self.SCALE_DEPOSIT_WAYPOINT[1] *= -1
            self.CUBE_PICKUP_1[1] *= -1
            self.CUBE_PICKUP_2[1] *= -1
            self.CUBE_PICKUP_ORIENTATION *= -1
            self.SCALE_INIT_WAYPOINT[1] *= -1


class SwitchScaleBase(OverallBase):

    @state(first=True)
    def decide_objectives(self):
        # first call, decide objective list
        if (self.current_side == self.fms_scale
           or not self.current_side == self.fms_switch):
            self.second_objective = 'switch'
            if not self.current_side == self.fms_scale:
                self.next_state_now('cross_to_scale')
                self.last_objective = 'scale'
            else:
                self.next_state_now('go_to_scale')
                if self.current_side == self.fms_switch:
                    self.last_objective = 'scale'
                else:
                    self.last_objective = 'switch'
        else:
            self.second_objective = 'scale'
            self.last_objective = 'scale'
            self.CUBE_PICKUP_2[1] = -self.CUBE_PICKUP_2[1]
            self.next_state_now('drive_by_switch')

    @state
    def drive_by_switch(self, initial_call, state_tm):
        if initial_call:
            self.switch_smooth = [self.DRIVE_BY_SWITCH_POINT[0]+0.5,
                                  self.DRIVE_BY_SWITCH_POINT[1]]
            self.motion.set_trajectory([
                self.current_waypoint,
                self.DRIVE_BY_SWITCH_POINT,
                self.switch_smooth], end_heading=self.DRIVE_BY_ORIENTATION,
                end_speed=self.DRIVE_BY_SPEED, motion_params=self.CUBE_RUN_MOTION)
            self.lifter_automation.engage(initial_state='move_switch')
            self.cube_number += 1
        if self.chassis.odometry_x > self.DRIVE_BY_SWITCH_POINT[0]:
            self.intake_automation.engage(initial_state='eject_cube', force=True)
            self.next_state_now('switch_to_cube')

    @state
    def switch_to_cube(self, initial_call):
        if initial_call:
            self.cube = self.CUBE_PICKUP_1
            pickup_waypoint = [self.PICKUP_WAYPOINT_X+0.3, self.cube[1]]
            pickup_waypoint_2 = [self.PICKUP_WAYPOINT_X-0.2, self.cube[1]]
            self.motion.set_trajectory([
                self.current_waypoint,
                self.switch_smooth,
                self.SWITCH_TO_CUBE_POINT,
                pickup_waypoint,
                pickup_waypoint_2],
                end_heading=self.CUBE_PICKUP_ORIENTATION,
                start_speed=self.chassis.speed,
                end_speed=0.5,
                motion_params=(2, 1, 1),
                smooth=False,
                waypoint_corner_radius=0.2)
            self.reset_mechanisms = False
        if self.motion.waypoint_idx == 1 and not self.reset_mechanisms:
            self.reset_mechanisms = True
            self.lifter_automation.engage(initial_state='reset', force=True)
        if not self.motion.trajectory_executing:
            print(f'chassis odom x {self.chassis.odometry_x} y {self.chassis.odometry_y}')
            self.next_state_now('pick_up_cube')

    def next_objective(self):
        if self.cube_number == 1:
            if self.second_objective == 'switch':
                self.next_state_now('deposit_switch')
            else:
                self.next_state_now('go_to_scale')
        else:
            if self.last_objective == 'switch':
                self.next_state_now('deposit_switch')
            else:
                self.next_state_now('go_to_scale')

    @state
    def deposit_switch(self, initial_call):
        """Deposit the cube on the switch."""
        self.chassis.set_inputs(0, 0, 0)
        if initial_call:
            self.cube_number += 1
            self.cube_inside = False
            self.lifter_automation.engage(initial_state='move_switch')
        if not self.lifter_automation.is_executing and not initial_call:
            self.intake_automation.engage(initial_state='eject_cube')
            self.next_state_now('wait_for_deposit')

    @state
    def wait_for_deposit(self, initial_call, state_tm):
        if not initial_call:
            if not self.intake_automation.is_executing:
                self.lifter.reset()
                self.next_state_now('nav_to_cube')


class LeftSwitchScale(SwitchScaleBase):
    MODE_NAME = 'Left Switch & Scale'

    def on_enable(self):
        super().on_enable()
        self.start_side = 'L'
        self.current_side = self.start_side
        self.done_switch = False
        self.cube_inside = True

        self.chassis.odometry_y = self.START_Y_COORDINATE

        if self.fms_switch == 'R':
            self.SWITCH_DEPOSIT[1] *= -1
            self.SWITCH_DEPOSIT_ORIENTATION *= -1
            self.CUBE_PICKUP_1[1] *= -1
            self.CUBE_PICKUP_2[1] *= -1
            self.CUBE_PICKUP_ORIENTATION *= -1
            self.DRIVE_BY_SWITCH_POINT[1] *= -1
            self.DRIVE_BY_ORIENTATION *= -1
            self.SWITCH_TO_CUBE_POINT[1] *= -1
        if self.fms_scale == 'R':
            self.SCALE_DEPOSIT[1] *= -1
            self.SCALE_DEPOSIT_WAYPOINT[1] *= -1
            self.SCALE_INIT_WAYPOINT[1] *= -1


class RightSwitchScale(SwitchScaleBase):
    MODE_NAME = 'Right Switch & Scale'

    def on_enable(self):
        super().on_enable()
        self.CROSS_POINT, self.OPP_CROSS_POINT = self.OPP_CROSS_POINT, self.CROSS_POINT
        self.chassis.odometry_y = -self.START_Y_COORDINATE

        print("FMS Switch %s" % self.fms_switch)
        print("FMS Scale %s" % self.fms_scale)
        if self.fms_switch == 'R':
            print("In right fms switch block")
            self.SWITCH_DEPOSIT[1] *= -1
            self.SWITCH_DEPOSIT_ORIENTATION *= -1
            self.CUBE_PICKUP_1[1] *= -1
            self.CUBE_PICKUP_2[1] *= -1
            # self.CUBE_PICKUP_ORIENTATION *= -1
            self.DRIVE_BY_SWITCH_POINT[1] *= -1
            self.DRIVE_BY_ORIENTATION *= -1
            self.SWITCH_TO_CUBE_POINT[1] *= -1
        if self.fms_scale == 'R':
            self.SCALE_DEPOSIT[1] *= -1
            self.SCALE_DEPOSIT_WAYPOINT[1] *= -1
            self.SCALE_INIT_WAYPOINT[1] *= -1

        self.start_side = 'R'
        self.current_side = self.start_side
        self.done_switch = False
        self.cube_inside = True
