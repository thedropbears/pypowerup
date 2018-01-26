import math
import numpy as np
from pyswervedrive.swervechassis import SwerveChassis
from utilities.bno055 import BNO055
from utilities.vector_pursuit import VectorPursuit


class ChassisMotion:

    chassis: SwerveChassis
    bno055: BNO055

    def __init__(self):
        self.enabled = False
        self.pursuit = VectorPursuit()

    def setup(self):
        self.pursuit.set_motion_params(3.0, 4.5, -4.5)

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

    def disable(self):
        self.enabled = False

    def on_enable(self):
        pass

    def execute(self):
        if self.enabled:
            self.chassis.field_oriented = True
            self.chassis.hold_heading = True

            odom_pos = np.array([self.chassis.odometry_x, self.chassis.odometry_y])
            odom_vel = np.array([self.chassis.odometry_x, self.chassis.odometry_y])

            speed = np.linalg.norm(odom_vel)

            direction_of_motion, speed_sp, over = self.pursuit.get_output(odom_pos, speed)

            vx = speed_sp * math.cos(direction_of_motion)
            vy = speed_sp * math.sin(direction_of_motion)

            # self.chassis.set_inputs(vx, vy, 0.0)
            self.chassis.set_velocity_heading(vx, vy, self.waypoints[self.waypoint_idx+1][2])

            if over:
                self.enabled = False

    @property
    def waypoint_idx(self):
        return self.pursuit.segment_idx
