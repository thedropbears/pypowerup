"""steps:
1 given way points set current way point to first way point
also define start and end point
3 calculate scale to check if gone over segment
4 calculate projected point
5 calculate look ahead point
6 calculate angle of look ahead from oreintation
7 output vector of angle and speed
"""
import numpy as np
import math


class VectorPursuit:

    def set_waypoints(self, waypoints: np.array):
        """Add way points to.
        Args:
            waypoint list of numpy arrays.
        """
        self.waypoints = waypoints
        self.segment = self.waypoints[1] - self.waypoints[0]
        self.segment_start = self.waypoints[0]
        self.segment_end = self.waypoints[1]
        self.waypoint_idx = 0

    def get_output(self, position: np.ndarray, orientation: int, speed: int):
        """Compute the angle to move the robot in to converge with waypoints.
        Args:
            position current robot position
            orientation of robot in radians
            speed in m/s of robot

        Returns:
            A vector of speed and direction based off robot's orientation.
        """

        # check if at edge of segment
        displacement = position - self.segment_start
        scale = displacement.dot(self.segment) / self.segment.dot(self.segment)

        # calculate projected point
        projected_point = self.segment_start + scale * self.segment
        # print(projected_point)

        # define look ahead distance
        look_ahead_distance = 1 + 0.3 * speed

        segment_normalised = self.segment / np.linalg.norm(self.segment)
        look_ahead_point = (projected_point
                            + segment_normalised * look_ahead_distance)
        print("Look ahead point: %s, projected_point %s"
              % (look_ahead_point, projected_point))

        # calculate angle of look ahead from oreintation
        new_x, new_y = look_ahead_point - position
        theta = math.atan2(new_y, new_x)
        angle = theta - orientation

        if scale > 1 and self.waypoint_idx < len(self.waypoints)-2:
            self.waypoint_idx += 1
            self.segment_end = self.waypoints[self.waypoint_idx+1]
            self.segment_start = self.waypoints[self.waypoint_idx]
            self.segment = self.segment_end - self.segment_start

        return angle

        # return vx, vy
