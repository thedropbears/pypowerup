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
        pass

    def set_waypoints(self, waypoints: np.ndarray):
        """ Pass as set of waypoints for the chassis to follow.

        Args:
            waypoints: A numpy array of waypoints that the chassis will follow.
                Waypoints are themselves arrays, constructed as follows:
                [x_in_meters, y_in_meters, orientation_in_radians, speed_in_meters]
        """
        self.waypoints = waypoints
        waypoints_xy = np.array([[waypoint[0], waypoint[1]] for waypoint in waypoints])
        self.pursuit.set_waypoints(waypoints_xy)
        self.enabled = True

    def disable(self):
        self.enabled = False

    def on_enable(self):
        pass

    def execute(self):
        if self.enabled:
            self.chassis.set_heading_sp(self.waypoints[self.waypoint_idx][2])
            self.chassis.field_oriented = False

            odom_pos = np.array([self.chassis.odometry_x, self.chassis.odometry_y])
            odom_vel = np.array([self.chassis.odometry_x, self.chassis.odometry_y])

            speed = np.linalg.norm(odom_vel)

            direction_of_motion = self.pursuit.get_output(odom_pos, self.bno055.getAngle(), speed)

            # TODO: control this later
            speed_sp = self.waypoints[self.waypoint_idx][3]

            vx = speed_sp * math.cos(direction_of_motion)
            vy = speed_sp * math.sin(direction_of_motion)

            self.chassis.set_inputs(vx, vy, 0.0)

    @property
    def waypoint_idx(self):
        return self.pursuit.waypoint_idx
