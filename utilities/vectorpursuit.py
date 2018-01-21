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

    def set_waypoint(self, waypoint: np.array):
        """Add way points to.
        Args:
            waypoint list of numpy arrays.
        """
        self.way_points = waypoint
        self.segment = self.way_points[0] - self.way_points[1]
        self.segment_start = self.way_points[0]
        self.segment_end = self.way_points[1]

    def get_output(self, position: np.array, orientation: int, speed: int):
        """Compute the angle to move the robot in to converge with waypoints.
        Args:
            position current robot position
            orientation of robot in radians
            speed in m/s of robot

        Returns:
            A vector of speed and direction based off robot's orientation.
        """

        # if set_waypoints hasn't been called raise error
        try:
            self.way_points
        except NameError:
            print("self.way_points not defined")
            quit()

        # check if at edge of segment
        displacement = position - self.segment_start
        scale = displacement.dot(self.segment) / self.segment.dot(self.segment)

        if scale > 1:
            self.segment_end = self.way_points[self.way_points.index(self.segment) + 1]
            self.segment_start = self.way_points[self.way_points.index(self.segment) + 1]
            self.segment = self.segment_end - self.segment_start
        else:
            self.segment_start = self.way_points[0]
            self.segment_end = self.way_points[1]
            self.segment = self.segment_end - self.segment_start

        # calculate projected point
        projected_point = self.segment + scale * self.segment
        print(projected_point)

        # define look ahead distance
        look_ahead_distance = 1 + 0.3 * speed

        # calculate look ahead point
        new_scale = (self.segment_end - self.segment_start) / \
                    (math.hypot(self.segment_end[0] -
                     self.segment_start[0], self.segment_end[1]
                     - self.segment_start[1]))
        look_ahead_point = projected_point + new_scale * \
            look_ahead_distance

        # calculate angle of look ahead from oreintation
        new_x, new_y = look_ahead_point - position
        theta = math.atan(new_y / new_x)
        angle = theta - orientation

        # output vector of angle and speed
        vx = speed * math.cos(angle)
        vy = speed * math.sin(angle)

        return vx, vy
