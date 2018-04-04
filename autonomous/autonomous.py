
"""The autonomous controls for the robot."""
import math

import numpy as np
import wpilib
from magicbot.state_machine import AutonomousStateMachine, state

from automations.cube import CubeManager
from automations.motion import ChassisMotion
from components.intake import Intake
from components.vision import Vision
from pyswervedrive.swervechassis import SwerveChassis
from utilities.imu import IMU

from wpilib import SmartDashboard


def reflect_2d_y(v: tuple) -> tuple:
    return (v[0], -v[1])


class OverallBase(AutonomousStateMachine):
    """statemachine designed to intelegently respond to possible situations in auto"""

    vision: Vision
    intake: Intake
    imu: IMU
    chassis: SwerveChassis
    ds: wpilib.DriverStation

    # automations
    motion: ChassisMotion
    cubeman: CubeManager

    START_Y_COORDINATE = 3.4 - SwerveChassis.WIDTH/2

    # Coordinates of various objectives no the field
    # Default to those for LEFT HAND SIDE of the field
    # TODO: determine how far forward/back of this we want to go
    SCALE_DEPOSIT = (7.6, 1.75)
    SCALE_DEPOSIT_WAYPOINT = (6, 1.9)
    # CUBE_PICKUP_1 = (6.2, 1.7)
    CUBE_PICKUP_1 = (6.2, 0.9)
    CUBE_PICKUP_2 = (6.2, 1.03)
    SWITCH_DEPOSIT = (5 + SwerveChassis.LENGTH/2, 1.2)
    SCALE_INIT_WAYPOINT = (6, 3)

    SWITCH_DEPOSIT_ORIENTATION = -math.pi
    SCALE_DEPOSIT_ORIENTATION = -math.pi / 20

    CUBE_PICKUP_ORIENTATION = -math.pi

    PICKUP_WAYPOINT_X = 6.5
    CROSS_POINT = (6.1, START_Y_COORDINATE)
    OPP_CROSS_POINT = (6.1, -START_Y_COORDINATE)
    DRIVE_BY_SWITCH_POINT = (4, 2 + SwerveChassis.LENGTH/2)
    """
    START_Y_COORDINATE = 1
    SCALE_DEPOSIT = (6 - SwerveChassis.LENGTH/2, 1)
    # CUBE_PICKUP_1 = (3 + SwerveChassis.LENGTH/2, 0.5)
    # CUBE_PICKUP_2 = (3 + SwerveChassis.LENGTH/2, -0.5)
    # CUBE_PICKUP_1 = (3+1, 0.5)
    CUBE_PICKUP_1 = (3+1, 0.5)
    CUBE_PICKUP_2 = (3+1, -0.5)
    SWITCH_DEPOSIT = (3 + SwerveChassis.LENGTH/2, 0)

    SWITCH_DEPOSIT_ORIENTATION = -math.pi

    CUBE_PICKUP_ORIENTATION = -math.pi

    PICKUP_WAYPOINT_X = 5
    SWITCH_TO_CUBE_POINT = (PICKUP_WAYPOINT_X, 0.8)

    DRIVE_BY_SWITCH_POINT = (2, 0.5 + SwerveChassis.LENGTH/2)
    CROSS_POINT = (4.5, 1)
    OPP_CROSS_POINT = (4.5, -1)
    """

    SWITCH_TO_CUBE_POINT = (PICKUP_WAYPOINT_X, 2.5)

    DRIVE_BY_ORIENTATION = -math.pi / 2
    DRIVE_BY_SPEED = 1

    CUBE_RUN_MOTION = (1, 1, 1)
    BACK_RUN_MOTION = (1, 1, 1)

    def init_plate_locations(self):
        """Initialise waypoints that depend on plate alignment."""
        if self.fms_scale == 'R':
            self.scale_init_waypoint = reflect_2d_y(self.SCALE_INIT_WAYPOINT)
            self.scale_deposit = reflect_2d_y(self.SCALE_DEPOSIT)
            self.scale_deposit_waypoint = reflect_2d_y(self.SCALE_DEPOSIT_WAYPOINT)
            self.scale_deposit_orientation = -self.SCALE_DEPOSIT_ORIENTATION
        else:
            self.scale_init_waypoint = self.SCALE_INIT_WAYPOINT
            self.scale_deposit = self.SCALE_DEPOSIT
            self.scale_deposit_waypoint = self.SCALE_DEPOSIT_WAYPOINT
            self.scale_deposit_orientation = self.SCALE_DEPOSIT_ORIENTATION

        if self.fms_switch == 'R':
            self.switch_deposit = reflect_2d_y(self.SWITCH_DEPOSIT)
            self.switch_deposit_orientation = -self.SWITCH_DEPOSIT_ORIENTATION
            self.switch_to_cube_point = reflect_2d_y(self.SWITCH_TO_CUBE_POINT)
            self.drive_by_switch_point = reflect_2d_y(self.DRIVE_BY_SWITCH_POINT)
            self.drive_by_orientation = -self.DRIVE_BY_ORIENTATION
        else:
            self.switch_deposit = self.SWITCH_DEPOSIT
            self.switch_deposit_orientation = self.SWITCH_DEPOSIT_ORIENTATION
            self.switch_to_cube_point = self.SWITCH_TO_CUBE_POINT
            self.drive_by_switch_point = self.DRIVE_BY_SWITCH_POINT
            self.drive_by_orientation = self.DRIVE_BY_ORIENTATION

    def init_cube_points(self, from_side: str):
        """Initialise the cube waypoints."""
        self.logger.info('Initialising cube waypoints: %s', from_side)
        if from_side == 'R':
            self.cube_pickup_1 = reflect_2d_y(self.CUBE_PICKUP_1)
            self.cube_pickup_2 = reflect_2d_y(self.CUBE_PICKUP_2)
            self.cube_pickup_orientation = -self.CUBE_PICKUP_ORIENTATION
        else:
            self.cube_pickup_1 = self.CUBE_PICKUP_1
            self.cube_pickup_2 = self.CUBE_PICKUP_2
            self.cube_pickup_orientation = self.CUBE_PICKUP_ORIENTATION

    def on_enable(self):
        super().on_enable()
        self.current_side = self.start_side

        game_data = self.ds.getGameSpecificMessage()
        self.fms_scale = game_data[1]  # L or R
        self.fms_switch = game_data[0]  # L or R
        self.init_plate_locations()

        self.chassis.odometry_x = SwerveChassis.LENGTH / 2

        self.cube_number = 0

        self.cubeman.start_match()
        self.imu.resetHeading()

        self.slow_scale = False
        self.done_switch = False
        self.cube_inside = True

        if self.start_side == 'L':
            self.chassis.odometry_y = self.START_Y_COORDINATE
            self.cross_point = self.CROSS_POINT
            self.opp_cross_point = self.OPP_CROSS_POINT
        else:
            self.chassis.odometry_y = -self.START_Y_COORDINATE
            self.cross_point = self.OPP_CROSS_POINT
            self.opp_cross_point = self.CROSS_POINT

    @state
    def nav_to_cube(self, initial_call):
        """Navigate on Dead Reckoning to the correct cube."""
        if initial_call:
            if self.cube_number == 1:
                self.cube = np.array(self.cube_pickup_1)
            elif self.cube_number >= 2:
                self.cube = np.array(self.cube_pickup_2)
            pickup_waypoint = (self.PICKUP_WAYPOINT_X, self.cube[1])
            if self.chassis.odometry_x > 6.0:
                self.motion.set_trajectory(
                    [self.current_waypoint, self.scale_deposit_waypoint, pickup_waypoint, self.cube],
                    end_heading=self.cube_pickup_orientation, end_speed=0.4,
                    # DO NOT SMOOTH WAYPONTS HERE (it breaks things)
                    # smooth=False, motion_params=self.CUBE_RUN_MOTION, wait_for_rotate=True)
                    smooth=False, motion_params=self.CUBE_RUN_MOTION, wait_for_rotate=False)
            else:
                self.motion.set_trajectory(
                    [self.current_waypoint, pickup_waypoint, self.cube],
                    end_heading=self.cube_pickup_orientation, end_speed=0.4,
                    # DO NOT SMOOTH WAYPONTS HERE (it breaks things)
                    smooth=False, motion_params=self.CUBE_RUN_MOTION)
        self.cubeman.start_intake()
        if not self.motion.trajectory_executing:
            self.next_state_now('pick_up_cube')

    @state
    def wait_for_intake(self):
        self.chassis.set_inputs(0, 0, 0)
        if not self.cubeman.is_executing:
            self.next_state_now('stop')

    @state
    def pick_up_cube(self, initial_call):
        """The robot rotates in the direction specified by the vision
        system while moving towards the cube. Combines two angles to find the absolute
        angle towards the cube"""

        if initial_call:
            self.cubeman.start_intake(force=True)
            self.pickup_start_pos = self.chassis.position
            self.seen_cube = False

        if self.intake.is_cube_contained() or self.intake.get_cube_distance() < 0.55:
            print("Intaking cube, finish vision navigation")
            self.next_state_now('stop')
            return

        vision_angle = self.vision.largest_cube()
        heading = self.imu.getAngle()
        if vision_angle is None:
            if initial_call:
                print('Initial call pick up cube, not seeing cube')
            self.chassis.set_velocity_heading(0.5, 0, self.cube_pickup_orientation)
            # if not self.seen_cube:
            #     self.chassis.set_inputs(0.0, 0, 0)
            # else:
            #     self.chassis.set_inputs(0.3, 0, 0)
            return
        else:
            self.seen_cube = True
        alignment_direction = heading + vision_angle * 1.7
        self.chassis.field_oriented = True

        # speed controller
        segment = self.cube - self.pickup_start_pos
        displacement = self.cube - self.chassis.position.reshape(2)
        total_dist = np.linalg.norm(segment)
        dist_along_segment = total_dist - np.linalg.norm(displacement)
        if dist_along_segment < 0:
            dist_along_segment = 0
        # slowly ramp down the speed as we approach the cube

        speed = 0.5
        SmartDashboard.putNumber('cube_pickup_speed', speed)
        SmartDashboard.putNumber('vision_angle', vision_angle)
        SmartDashboard.putNumber('vision_alignment_heading', alignment_direction)

        vx = speed*math.cos(alignment_direction)
        vy = speed*math.sin(alignment_direction)
        # TODO: change this to hold a heading via PID
        self.chassis.set_velocity_heading(vx, vy, self.cube_pickup_orientation)
        # self.chassis.set_inputs(vx, vy, 0)

    @state
    def stop(self, initial_call, state_tm):
        if initial_call:
            self.chassis.set_inputs(0, 0, 0)
        if (self.chassis.speed < 0.1 or state_tm > 1) and not self.cubeman.is_executing:
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
                    self.scale_init_waypoint,
                    self.scale_deposit_waypoint,
                    self.scale_deposit
                ], end_heading=self.scale_deposit_orientation)
            elif self.slow_scale:
                print('back run scale')
                self.motion.set_trajectory(
                    [self.current_waypoint, self.scale_deposit_waypoint, self.scale_deposit],
                    end_heading=self.scale_deposit_orientation,
                    motion_params=self.BACK_RUN_MOTION, smooth=True)
            else:
                self.motion.set_trajectory([
                    self.current_waypoint,
                    self.scale_deposit_waypoint,
                    self.scale_deposit
                ], end_heading=self.scale_deposit_orientation)
            if not self.slow_engage:
                self.cubeman.lift_to_scale(force=True)
        if self.motion.linear_position > 3 and self.slow_engage:
            self.cubeman.lift_to_scale(force=True)
        # if not self.motion.trajectory_executing and state_tm > 4:
        if not self.motion.trajectory_executing:
            self.next_state_now('deposit_scale')

    @state
    def cross_to_scale(self, initial_call, state_tm):
        if initial_call:
            self.motion.set_trajectory([
                self.current_waypoint,
                self.cross_point,
                self.opp_cross_point,
                self.scale_deposit
            ], end_heading=self.scale_deposit_orientation)
        if self.motion.linear_position > 6.5:
            self.cubeman.lift_to_scale()
        if not self.motion.trajectory_executing:
            self.next_state_now("deposit_scale")

    @state
    def deposit_scale(self, initial_call, state_tm):
        """Deposit the cube on the scale."""
        if initial_call:
            self.chassis.set_inputs(0, 0, 0)
            self.cube_number += 1
            self.cube_inside = False
            self.cubeman.eject(force=True)
            print("Ejecting Cube")
        if not self.cubeman.is_executing and state_tm > 0.04:
            self.cubeman.engage(initial_state='waiting_to_reset')
            self.next_state_now('nav_to_cube')

    @property
    def current_waypoint(self):
        return (self.chassis.odometry_x, self.chassis.odometry_y)


class DoubleScaleBase(OverallBase):
    def on_enable(self):
        super().on_enable()
        self.init_cube_points(self.fms_scale)

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
    start_side = 'L'


class RightDoubleScale(DoubleScaleBase):
    MODE_NAME = 'Right Double Scale'
    start_side = 'R'


class SwitchScaleBase(OverallBase):
    def on_enable(self):
        super().on_enable()
        self.init_cube_points(self.fms_scale)

    @state
    def decide_objectives(self):
        # first call, decide objective list
        if (self.current_side == self.fms_scale
           or self.current_side != self.fms_switch):
            self.second_objective = 'switch'
            if self.current_side != self.fms_scale:
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
            self.cube_pickup_2 = reflect_2d_y(self.cube_pickup_2)
            self.next_state_now('drive_by_switch')

    @state
    def drive_by_switch(self, initial_call, state_tm):
        if initial_call:
            self.motion.set_trajectory(
                [self.current_waypoint, self.drive_by_switch_point],
                end_heading=self.drive_by_orientation,
                motion_params=self.CUBE_RUN_MOTION)
            self.cube_number += 1
        if state_tm > 0.5:
            self.cubeman.lift_to_switch(force=True)
        if not self.motion.trajectory_executing:
            self.cubeman.eject(force=True)
            self.next_state('wait_for_switch')

    @state
    def wait_for_switch(self):
        if not self.cubeman.is_executing:
            self.next_state_now('switch_to_cube')

    @state
    def switch_to_cube(self, initial_call):
        if initial_call:
            self.cube = self.cube_pickup_1
            pickup_waypoint = (self.PICKUP_WAYPOINT_X+0.3, self.cube[1])
            pickup_waypoint_2 = (self.PICKUP_WAYPOINT_X-0.2, self.cube[1])
            self.motion.set_trajectory(
                [self.current_waypoint, self.switch_to_cube_point, pickup_waypoint, pickup_waypoint_2],
                end_heading=self.cube_pickup_orientation, end_speed=0.5,
                motion_params=(2, 1, 1),
                smooth=False, waypoint_corner_radius=0.2)
            self.reset_mechanisms = False
        if self.motion.waypoint_idx == 1 and not self.reset_mechanisms:
            self.reset_mechanisms = True
            self.cubeman.reset(force=True)
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
        if initial_call:
            self.chassis.set_inputs(0.5, 0, 0)
            self.cube_number += 1
            self.cube_inside = False
            self.cubeman.lift_to_switch(force=True)
        elif not self.cubeman.is_executing:
            print('eject cube')
            self.next_state('wait_for_deposit')

    @state
    def wait_for_deposit(self, initial_call, state_tm):
        self.chassis.set_inputs(0.0, 0, 0)
        if initial_call:
            self.cubeman.eject(force=True)
        elif not self.cubeman.is_executing:
            self.cubeman.reset(force=True)
            self.next_state_now('nav_to_cube')


class LeftSwitchScale(SwitchScaleBase):
    MODE_NAME = 'Left Switch & Scale'
    start_side = 'L'

    @state(first=True)
    def first_state(self):
        self.next_state_now('decide_objectives')


class RightSwitchScale(SwitchScaleBase):
    MODE_NAME = 'Right Switch & Scale'
    start_side = 'R'

    def on_enable(self):
        super().on_enable()
        # XXX this is just how the original code behaved
        self.cube_pickup_orientation = self.CUBE_PICKUP_ORIENTATION

    @state(first=True)
    def first_state(self):
        self.next_state_now('decide_objectives')


class SameSideBase(SwitchScaleBase):
    def on_enable(self):
        # Override the switch/scale base cube waypoints here.
        OverallBase.on_enable(self)
        self.init_cube_points(self.start_side)

    @state(first=True)
    def same_side_decide(self):
        if self.current_side == self.fms_scale == self.fms_switch:
            if self.priority == 'switch':
                self.second_objective = 'scale'
                self.last_objective = 'switch'
                self.next_state_now('drive_by_switch')
            else:
                self.next_state_now('decide_objectives')
            return
        elif self.current_side == self.fms_scale:
            self.second_objective = 'scale'
            self.last_objective = 'scale'
            self.next_state_now('go_to_scale')
            return
        elif self.current_side == self.fms_switch:
            self.second_objective = 'switch'
            self.last_objective = 'switch'
            self.next_state_now('drive_by_switch')
            return
        else:
            self.motion.set_trajectory([self.current_waypoint, (3, self.chassis.odometry_y)], 0)
            self.next_state_now('moving_forward')
            return

    @state
    def moving_forward(self):
        if not self.motion.trajectory_executing:
            self.done()


class LeftSameSide(SameSideBase):
    start_side = 'L'


class LeftSameSideSwitch(LeftSameSide):
    MODE_NAME = 'Left Same Side Switch'
    priority = 'switch'


class LeftSameSideScale(LeftSameSide):
    MODE_NAME = 'Left Same Side Scale'
    priority = 'scale'


class RightSameSide(SameSideBase):
    start_side = 'R'

    def on_enable(self):
        super().on_enable()
        # XXX this is just how the original code behaved
        self.cube_pickup_orientation = self.CUBE_PICKUP_ORIENTATION


class RightSameSideSwitch(RightSameSide):
    MODE_NAME = 'Right Same Side Switch'
    priority = 'switch'


class RightSameSideScale(RightSameSide):
    MODE_NAME = 'Right Same Side Scale'
    priority = 'scale'


class PickupCubeAuto(OverallBase):
    MODE_NAME = "Vision Pickup Test DON'T USE"
    start_side = 'L'  # dummy

    @state(first=True)
    def start(self):
        self.cube = (3, 0)
        self.chassis.odometry_y = 0
        self.chassis.odometry_x = 6
        self.next_state_now('pick_up_cube')

    @state
    def deposit_switch(self, initial_call):
        """Deposit the cube on the switch."""
        if initial_call:
            self.chassis.set_inputs(0.5, 0, 0)
            self.cube_number += 1
            self.cube_inside = False
            self.cubeman.lift_to_switch(force=True)
        elif not self.cubeman.is_executing:
            print('eject cube')
            self.next_state('wait_for_deposit')

    @state
    def wait_for_deposit(self, initial_call, state_tm):
        self.chassis.set_inputs(0.0, 0, 0)
        if initial_call:
            self.cubeman.eject(force=True)
        elif not self.cubeman.is_executing:
            self.cubeman.reset(force=True)
            self.done()

    def next_objective(self):
        self.next_state_now('deposit_switch')
        print('next obj called')
