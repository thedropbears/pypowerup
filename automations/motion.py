import math
import numpy as np
from pyswervedrive.swervechassis import SwerveChassis
from utilities.bno055 import BNO055
from utilities.vector_pursuit import VectorPursuit
from utilities.profile_generator import generate_interpolation_function
from wpilib import SmartDashboard
from networktables import NetworkTables


class ChassisMotion:

    chassis: SwerveChassis
    bno055: BNO055

    # heading motion feedforward/back gains
    kPh = 3  # proportional gain
    kVh = 1  # feedforward gain
    kIh = 0  # integral gain
    kDh = 10  # derivative gain

    heading_adjustment_proportion = 0.6

    def __init__(self):
        self.enabled = False
        self.pursuit = VectorPursuit()

    def setup(self):
        self.pursuit.set_motion_params(4.0, 4, -3)

    def set_waypoints(self, waypoints: np.ndarray):
        """ Pass as set of waypoints for the chassis to follow.

        Args:
            waypoints: A numpy array of waypoints that the chassis will follow.
                Waypoints are themselves arrays, constructed as follows:
                [x_in_meters, y_in_meters, orientation_in_radians, speed_in_meters]
        """
        self.waypoints = waypoints
        self.pursuit.set_waypoints(waypoints)
        self.enabled = True
        self.chassis.heading_hold_on()
        self.update_heading_profile()
        self.heading_error_i = 0

    def update_heading_profile(self):
        self.current_seg_distance = np.linalg.norm(self.pursuit.segment)
        heading_start = self.bno055.getAngle()
        heading_end = self.waypoints[self.waypoint_idx+1][2]
        self.heading_profile_function = generate_interpolation_function(
                heading_start, heading_end, self.current_seg_distance*self.heading_adjustment_proportion)
        self.last_heading_error = 0

    def disable(self):
        self.enabled = False

    def on_enable(self):
        pass

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

            if seg_end_dist < self.current_seg_distance*self.heading_adjustment_proportion:
                heading_seg = self.heading_profile_function(seg_end_dist, speed)
            else:
                heading_seg = (self.waypoints[self.waypoint_idx+1][2], 0, 0)

            # get the current heading of the robot since last reset
            heading = self.bno055.getRawHeading() - self.bno055.offset
            # calculate the heading error
            heading_error = heading_seg[0] - heading
            # wrap heading error, stops jumping by 2 pi from the gyro
            heading_error = math.atan2(math.sin(heading_error),
                                       math.cos(heading_error))
            # sum the heading error over the timestep
            self.heading_error_i += heading_error
            # calculate the derivative of the heading error
            d_heading_error = (heading_error - self.last_heading_error)

            # generate the rotational output to the chassis
            heading_output = (
                self.kPh * heading_error + self.kVh * heading_seg[1]
                + self.heading_error_i*self.kIh + d_heading_error*self.kDh)

            # store the current errors to be used to compute the
            # derivatives in the next timestep
            self.last_heading_error = heading_error

            vx = speed_sp * math.cos(direction_of_motion)
            vy = speed_sp * math.sin(direction_of_motion)

            # self.chassis.set_velocity_heading(vx, vy, self.waypoints[self.waypoint_idx+1][2])
            self.chassis.set_inputs(vx, vy, heading_output)

            SmartDashboard.putNumber('vector_pursuit_heading', direction_of_motion)
            SmartDashboard.putNumber('vector_pursuit_speed', speed_sp)
            NetworkTables.flush()

            if over:
                print("Motion over")
                self.enabled = False

    @property
    def waypoint_idx(self):
        return self.pursuit.segment_idx
