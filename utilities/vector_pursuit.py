"""steps:
1 given way points set current way point to first way point
also define start and end point
3 calculate scale to check if gone over segment
4 calculate projected point
5 calculate look ahead point
6 calculate angle of look ahead from oreintation
7 output vector of angle and speed
"""
import math
from utilities.profile_generator import generate_trapezoidal_function
import numpy as np


class VectorPursuit:

    def set_waypoints(self, waypoints: np.array):
        """Add way points to.
        Args:
            waypoint list of numpy arrays.
        """
        self.waypoints = waypoints
        self.waypoints_xy = np.array([[waypoint[0], waypoint[1]] for waypoint in self.waypoints])
        self.segment_idx = None
        self.increment_segment()

    def set_motion_params(self, top_speed, top_accel, top_decel):
        self.top_speed = top_speed
        self.top_accel = top_accel
        self.top_decel = top_decel

    def increment_segment(self):
        if self.segment_idx is None:
            self.segment_idx = 0
        else:
            self.segment_idx += 1
        self.segment = (self.waypoints_xy[self.segment_idx+1]
                        - self.waypoints_xy[self.segment_idx])
        start_speed = self.waypoints[self.segment_idx][3]
        end_speed = self.waypoints[self.segment_idx+1][3]
        seg_length = np.linalg.norm(self.segment)
        self.speed_function = generate_trapezoidal_function(
                0, start_speed, seg_length, end_speed,
                self.top_speed, self.top_accel, self.top_decel)

    def get_output(self, position: np.ndarray, speed: float):
        """Compute the angle to move the robot in to converge with waypoints.
        Args:
            position current robot position
            speed in m/s of robot

        Returns:
            A vector of speed and direction based off robot's orientation.
        """

        # check if at edge of segment
        displacement = position - self.waypoints_xy[self.segment_idx]
        scale = displacement.dot(self.segment) / self.segment.dot(self.segment)

        # calculate projected point
        projected_point = (self.waypoints_xy[self.segment_idx]
                           + scale * self.segment)

        dist_to_end = np.linalg.norm(self.segment) - np.linalg.norm(self.waypoints_xy[self.segment_idx+1] - position)
        if dist_to_end < 0:
            dist_to_end = 0
        speed_sp = self.speed_function(dist_to_end)

        # define look ahead distance
        look_ahead_distance = 0.1 + 0.3 * speed

        look_ahead_point = projected_point
        look_ahead_remaining = look_ahead_distance
        look_ahead_waypoint = self.segment_idx
        while look_ahead_remaining > 0:
            segment_start = self.waypoints_xy[look_ahead_waypoint]
            segment_end = self.waypoints_xy[look_ahead_waypoint+1]
            segment = segment_end - segment_start
            segment_normalised = segment / np.linalg.norm(segment)
            look_ahead_point = (projected_point + look_ahead_remaining
                                * segment_normalised)
            if look_ahead_waypoint == len(self.waypoints)-2:
                break
            projected_point = segment_end
            look_ahead_waypoint += 1

        segment_normalised = self.segment / np.linalg.norm(self.segment)

        # calculate angle of look ahead from oreintation
        new_x, new_y = look_ahead_point - position
        theta = math.atan2(new_y, new_x)

        next_seg = False
        if scale > 1 and self.segment_idx < len(self.waypoints)-2:
            self.increment_segment()
            next_seg = True

        over = False
        if np.linalg.norm(position - self.waypoints_xy[-1]) < 0.1:
            over = True

        return theta, speed_sp, next_seg, over
