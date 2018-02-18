import math
import numpy as np
from pyswervedrive.swervechassis import SwerveChassis
from utilities.navx import NavX
from utilities.vector_pursuit import VectorPursuit
from utilities.profile_generator import generate_trapezoidal_trajectory
from utilities.functions import constrain_angle
from wpilib import SmartDashboard


class ChassisMotion:

    chassis: SwerveChassis
    imu: NavX

    # heading motion feedforward/back gains
    kPh = 4  # proportional gain
    kVh = 1  # feedforward gain
    kIh = 0  # integral gain
    kDh = 0  # derivative gain

    def __init__(self):
        self.enabled = False
        self.pursuit = VectorPursuit()
        self.last_heading_error = 0

    def setup(self):
        self.pursuit.set_motion_params(4, 4, -3)
        # self.pursuit.set_motion_params(2, 2, -2)

    def set_waypoints(self, waypoints: np.ndarray):
        """ Pass as set of waypoints for the chassis to follow.

        Args:
            waypoints: A numpy array of waypoints that the chassis will follow.
                Waypoints are themselves arrays, constructed as follows:
                [x_in_meters, y_in_meters, orientation_in_radians, speed_in_meters]
        """
        self.waypoints = waypoints
        print("Motion waypoints %s" % self.waypoints)
        self.pursuit.set_waypoints(waypoints)
        self.enabled = True
        self.chassis.heading_hold_on()
        self.update_heading_profile()
        self.heading_error_i = 0

    def update_heading_profile(self):
        self.current_seg_distance = np.linalg.norm(self.pursuit.segment)
        heading_end = self.waypoints[self.waypoint_idx+1][2]
        self.heading_profile = generate_trapezoidal_trajectory(self.imu.getAngle(), 0,
                                                               heading_end, 0, 3, 3, -3, 50)
        self.last_heading_error = 0

    def disable(self):
        self.enabled = False

    def on_enable(self):
        self.waypoints = []
        self.enabled = False

    def execute(self):
        if self.enabled:
            self.chassis.field_oriented = True
            self.chassis.hold_heading = False

            odom_pos = np.array([self.chassis.odometry_x, self.chassis.odometry_y])
            odom_vel = np.array([self.chassis.odometry_x_vel, self.chassis.odometry_y_vel])

            speed = np.linalg.norm(odom_vel)
            # print("Odom speed %s" % speed)

            direction_of_motion, speed_sp, next_seg, over = self.pursuit.get_output(odom_pos, speed)

            if next_seg:
                self.update_heading_profile()
            seg_end = self.pursuit.waypoints_xy[self.waypoint_idx+1]
            seg_end_dist = (np.linalg.norm(self.pursuit.segment)
                            - np.linalg.norm(seg_end - odom_pos))
            if seg_end_dist < 0:
                seg_end_dist = 0

            if self.heading_profile:
                heading_seg = self.heading_profile.pop(0)
            else:
                heading_seg = (self.waypoints[self.waypoint_idx+1][2], 0, 0)

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
                self.kPh * heading_error + self.kVh * heading_seg[1]
                + self.heading_error_i*self.kIh + d_heading_error*self.kDh)
            # print("Heading output %s" % heading_output)

            # store the current errors to be used to compute the
            # derivatives in the next timestep
            self.last_heading_error = heading_error
            vx = speed_sp * math.cos(direction_of_motion)
            vy = speed_sp * math.sin(direction_of_motion)

            # self.chassis.set_velocity_heading(vx, vy, self.waypoints[self.waypoint_idx+1][2])
            self.chassis.set_inputs(vx, vy, heading_output)

            SmartDashboard.putNumber('vector_pursuit_heading', direction_of_motion)
            SmartDashboard.putNumber('vector_pursuit_speed', speed_sp)
            SmartDashboard.putNumber('heading_mp_sp', heading_seg[0])
            SmartDashboard.putNumber('heading_mp_error', heading_error)

            if over:
                print("Motion over")
                self.enabled = False
                if self.waypoints[-1][3] == 0:
                    self.chassis.set_inputs(0, 0, 0)

    @property
    def waypoint_idx(self):
        return self.pursuit.segment_idx
