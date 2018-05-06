import math
import time

import numpy as np
from wpilib import SmartDashboard

from pyswervedrive.chassis import Chassis
from utilities.imu import IMU
from utilities.vector_pursuit import VectorPursuit
from utilities.profile_generator import generate_trapezoidal_function, smooth_waypoints
from utilities.functions import constrain_angle


class ChassisMotion:

    chassis: Chassis
    imu: IMU

    # heading motion feedforward/back gains
    kPh = 3  # proportional gain
    kVh = 1  # feedforward gain
    kIh = 0  # integral gain
    kDh = 0  # derivative gain
    kAh = 0.2

    kP = 0.5
    kI = 0
    kD = 0
    kV = 1
    kA = 0.2

    waypoint_corner_radius = 0.7

    def __init__(self):
        self.enabled = False
        self.pursuit = VectorPursuit()
        self.last_heading_error = 0

    def setup(self):
        pass

    def set_trajectory(self, waypoints: np.ndarray, end_heading,
                       start_speed=0.0, end_speed=0.25, smooth=True,
                       motion_params=(2.5, 2, 1.5), waypoint_corner_radius=None, wait_for_rotate=False):
        """ Pass as set of waypoints for the chassis to follow.

        Args:
            waypoints: A numpy array of waypoints that the chassis will follow.
                Waypoints are themselves arrays, constructed as follows:
                [x_in_meters, y_in_meters]
        """
        print(f'original_waypoints {waypoints}')
        if smooth:
            if waypoint_corner_radius is None:
                waypoint_corner_radius = self.waypoint_corner_radius
            waypoints_smoothed = smooth_waypoints(waypoints, radius=waypoint_corner_radius)
        else:
            waypoints_smoothed = [np.array(point) for point in waypoints]
        print(f'smoothed_waypoints {waypoints_smoothed}')
        print(f'end_heading {end_heading}')
        trajectory_length = sum([np.linalg.norm(waypoints_smoothed[i] - waypoints_smoothed[i-1])
                                 for i in range(1, len(waypoints_smoothed))])
        self.end_heading = end_heading
        self.end_distance = trajectory_length
        self.start_segment_tm = time.monotonic()

        self.update_linear_profile(motion_params, start_speed, end_speed)
        self.update_heading_profile()

        self.waypoints = waypoints_smoothed
        self.pursuit.set_waypoints(waypoints_smoothed)

        self.wait_for_rotate = wait_for_rotate
        self.started_moving = False
        print(f'Wait for rotate {self.wait_for_rotate}')

        self.enabled = True
        if self.distance_traj_tm < self.heading_traj_tm:
            print(f'WARNING: Heading trajectory ({self.heading_traj_tm}s) longer than linear trajectory ({self.distance_traj_tm}s)')

        self.chassis.heading_hold_off()

    def update_linear_profile(self, motion_params, start_speed, end_speed):
        self.speed_function, self.distance_traj_tm = generate_trapezoidal_function(
                                                            0, start_speed,
                                                            self.end_distance,
                                                            end_speed,
                                                            v_max=motion_params[0],
                                                            a_pos=motion_params[1],
                                                            a_neg=motion_params[2])
        self.linear_position = 0
        self.last_position = self.chassis.position
        print(f'start_position {self.last_position}')
        self.last_linear_error = 0
        self.linear_error_i = 0

    def update_heading_profile(self):
        heading = self.imu.getAngle()
        # this ensures we always rotate the same way going from the switch to
        # the scale, but that we do not do 360s while maniuplating cubes at the
        # switch
        delta = constrain_angle(self.end_heading - heading)
        end_heading = heading + delta
        self.heading_function, self.heading_traj_tm = generate_trapezoidal_function(
                                                            heading, 0, end_heading, 0,
                                                            v_max=3, a_pos=2, a_neg=2)
        self.last_heading_error = 0
        self.heading_error_i = 0

    @property
    def trajectory_executing(self):
        return self.enabled

    @property
    def average_speed(self):
        speed = 0
        for module in self.chassis.modules:
            speed += module.wheel_vel / 4
        return speed

    def disable(self):
        self.enabled = False

    def on_enable(self):
        self.waypoints = []
        self.enabled = False

    def execute(self):
        if self.enabled:
            self.chassis.field_oriented = True
            self.chassis.hold_heading = False
            # TODO: re-enable if we end up not using callback method
            # self.chassis.update_odometry()

            direction_of_motion, next_seg, over = self.pursuit.get_output(self.chassis.position,
                                                                          self.chassis.speed)

            speed_sp = self.run_speed_controller()
            heading_output, heading_error = self.run_heading_controller()

            vx = speed_sp * math.cos(direction_of_motion)
            vy = speed_sp * math.sin(direction_of_motion)

            if not self.started_moving:
                if self.chassis.all_aligned:
                    self.started_moving = True
                else:
                    self.chassis.set_inputs(vx/100, vy/100, heading_output/100)

            self.chassis.set_inputs(vx, vy, heading_output)

            SmartDashboard.putNumber('vector_pursuit_heading', direction_of_motion)
            SmartDashboard.putNumber('vector_pursuit_speed', speed_sp)

            if over:
                if not self.wait_for_rotate:
                    print(f"Motion over at {self.chassis.position}, heading {self.imu.getAngle()}")
                    self.enabled = False
                    self.chassis.set_inputs(0, 0, 0)
                else:
                    if abs(heading_error) < 0.1:
                        print(f"Motion over at {self.chassis.position}, heading {self.imu.getAngle()}")
                        self.enabled = False
                        self.chassis.set_inputs(0, 0, 0)

    def run_speed_controller(self):
        chassis_pos = self.chassis.position
        self.linear_position += np.linalg.norm(chassis_pos - self.last_position)

        profile_tm = time.monotonic() - self.start_segment_tm
        if profile_tm > self.distance_traj_tm:
            return 0.25

        linear_seg = self.speed_function(profile_tm)
        if linear_seg is None:
            linear_seg = (0, 0, 0)
            print("WARNING: Linear segment is 0")

        SmartDashboard.putNumber("vector_pursuit_position", self.linear_position)
        # calculate the position errror
        pos_error = linear_seg[0] - self.linear_position
        # calucate the derivative of the position error
        self.d_pos_error = (pos_error - self.last_linear_error)
        # sum the position error over the timestep
        self.linear_error_i += pos_error

        # generate the linear output to the chassis (m/s)
        speed_sp = (self.kP*pos_error + self.kV*linear_seg[1]
                    + self.kA*linear_seg[2] + self.kI*self.linear_error_i
                    + self.kD*self.d_pos_error)

        self.last_position = chassis_pos
        self.last_linear_error = pos_error

        SmartDashboard.putNumber('linear_mp_sp', linear_seg[0])
        SmartDashboard.putNumber('linear_mp_error', pos_error)
        SmartDashboard.putNumber('linear_pos', self.linear_position)

        return speed_sp

    def run_heading_controller(self):
        if self.heading_function is not None:
            heading_time = time.monotonic() - self.start_segment_tm
            if heading_time < self.heading_traj_tm:
                heading_seg = self.heading_function(heading_time)
            else:
                self.heading_function = None
                heading_seg = (self.end_heading, 0, 0)
        else:
            heading_seg = (self.end_heading, 0, 0)

        # get the current heading of the robot since last reset
        # getRawHeading has been swapped for getAngle
        heading = self.imu.getAngle()
        # calculate the heading error
        heading_error = heading_seg[0] - heading
        # wrap heading error, stops jumping by 2 pi from the imu
        heading_error = constrain_angle(heading_error)
        # sum the heading error over the timestep
        self.heading_error_i += heading_error
        # calculate the derivative of the heading error
        d_heading_error = (heading_error - self.last_heading_error)

        # generate the rotational output to the chassis
        heading_output = (
            self.kPh * heading_error + self.kAh * heading_seg[2] + self.kVh * heading_seg[1]
            + self.heading_error_i*self.kIh + d_heading_error*self.kDh)

        # store the current errors to be used to compute the
        # derivatives in the next timestep
        self.last_heading_error = heading_error

        SmartDashboard.putNumber('heading_mp_sp', heading_seg[0])
        SmartDashboard.putNumber('heading_mp_error', heading_error)

        return heading_output, heading_error

    @property
    def waypoint_idx(self):
        return self.pursuit.segment_idx
