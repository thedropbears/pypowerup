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
        self.segment_idx = 0

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
        displacement = position - self.waypoints[self.segment_idx]
        scale = displacement.dot(self.segment) / self.segment.dot(self.segment)

        # calculate projected point
        projected_point = (self.waypoints[self.segment_idx]
                           + scale * self.segment)
        # print(projected_point)

        # define look ahead distance
        look_ahead_distance = 0.1 + 0.1 * speed

        look_ahead_point = projected_point
        look_ahead_remaining = look_ahead_distance
        look_ahead_waypoint = self.segment_idx
        while look_ahead_remaining > 0:
            segment_start = self.waypoints[look_ahead_waypoint]
            segment_end = self.waypoints[look_ahead_waypoint+1]
            segment = segment_end - segment_start
            segment_normalised = segment / np.linalg.norm(segment)
            look_ahead_point = (projected_point + look_ahead_remaining
                                * segment_normalised)
            if look_ahead_waypoint == len(self.waypoints)-2:
                break
            projected_point = segment_end
            look_ahead_waypoint += 1

        segment_normalised = self.segment / np.linalg.norm(self.segment)
        # look_ahead_point = (projected_point
                           # + segment_normalised * look_ahead_distance)
        # print("Look ahead point: %s, projected_point %s"
            # % (look_ahead_point, projected_point))

        # calculate angle of look ahead from oreintation
        new_x, new_y = look_ahead_point - position
        theta = math.atan2(new_y, new_x)

        if scale > 1 and self.segment_idx < len(self.waypoints)-2:
            self.segment_idx += 1
            self.segment = (self.waypoints[self.segment_idx+1]
                            - self.waypoints[self.segment_idx])
            # print(self.segment_idx)
            # print(self.segment)

        over = False
        if np.linalg.norm(position - self.waypoints[-1]) < 0.2:
            over = True
            # print(over)

        return theta, over
