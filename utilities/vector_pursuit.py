import math
from utilities.profile_generator import generate_trapezoidal_function
import numpy as np


class VectorPursuit:

    def set_waypoints(self, waypoints: np.array):
        """Set the waypoints the controller should drive the robot through.

        Args:
            waypoints: A list of [x, y, orientation, speed] points to pass
                through.
                Note that this controller does not currently control
                orientation, only speed and position.
        """
        self.waypoints = waypoints
        self.waypoints_xy = np.array([[waypoint[0], waypoint[1]] for waypoint in self.waypoints])
        self.segment_idx = None
        self.increment_segment()

    def set_motion_params(self, top_speed, top_accel, top_decel):
        """Set the speed and acceleration limits the controller is to respect.

        Args:
            cruise_vel: Cruise velocity of the drivebase (top speed the
                controller will set). Units m/s.
            top_accel: Maximum acceleration the controller will set. Units
                m/s/s.
            top_decel: Maximum deceleration the controller will set. Must be
                less than 0. Units m/s/s.
        """
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
            position: Robot's position as an [x, y] numpy array. Units m.
            speed: Robot's speed as a scalar value (i.e. the norm of it's x, y
                velocity), must be >0. Units of m/s.
        Returns:
            float: Direction of motion to command the robot at. Units radians
                clockwise from the positive x axis.
            float: Speed of motion to command the robot at. Units of m/s.
            bool: Whether we moved to the next waypoint or not.
            bool: Wherther we finished executing the entire set trajectory.
        """

        # check if at edge of segment
        displacement = position - self.waypoints_xy[self.segment_idx]
        scale = displacement.dot(self.segment) / self.segment.dot(self.segment)

        # calculate projected point
        projected_point = (self.waypoints_xy[self.segment_idx]
                           + scale * self.segment)

        # calculate the norm of the vector to the end of our current trajectory
        dist_to_end = np.linalg.norm(self.segment) - np.linalg.norm(self.waypoints_xy[self.segment_idx+1] - position)
        if dist_to_end < 0:
            dist_to_end = 0
        speed_sp = self.speed_function(dist_to_end)

        # define look ahead distance
        look_ahead_distance = 0.1 + 0.3 * speed

        # iterate over the segments from our current projected position until
        # we exhaust the lookahead distance
        look_ahead_point = projected_point
        look_ahead_remaining = look_ahead_distance
        look_ahead_waypoint = self.segment_idx
        while look_ahead_remaining > 0:
            segment_start = self.waypoints_xy[look_ahead_waypoint]
            segment_end = self.waypoints_xy[look_ahead_waypoint+1]
            segment = segment_end - segment_start

            # unit vector of the segment we are iterating over
            segment_normalised = segment / np.linalg.norm(segment)

            look_ahead_point = (projected_point + look_ahead_remaining
                                * segment_normalised)

            # take away from the remaining look ahead the amount we have travelled
            # along the segment
            look_ahead_remaining -= np.linalg.norm(segment_end-projected_point)
            if look_ahead_waypoint == len(self.waypoints)-2:
                break

            # projected point is now the start of the next segment
            projected_point = segment_end
            look_ahead_waypoint += 1

        segment_normalised = self.segment / np.linalg.norm(self.segment)

        # calculate angle of look ahead from oreintation
        new_x, new_y = look_ahead_point - position
        theta = math.atan2(new_y, new_x)

        next_seg = False
        over = False
        if scale > 1 and self.segment_idx < len(self.waypoints)-2:
            self.increment_segment()
            next_seg = True
        elif (np.linalg.norm(position - self.waypoints_xy[-1]) < 0.1
              or (scale >= 0.95 and self.segment_idx == len(self.waypoints)-2)):
            over = True

        return theta, speed_sp, next_seg, over
