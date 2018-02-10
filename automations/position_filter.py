from components.vision import Vision
from pyswervedrive.swervechassis import SwerveChassis
from utilities.navx import NavX
from utilities.functions import constrain_angle
from utilities.kalman import Kalman

from collections import deque
import math
import numpy as np
import time


class PositionFilter:

    imu: NavX
    chassis: SwerveChassis
    vision: Vision

    # initial covariance of the system
    P_init = np.identity(2, dtype=float)*1e-4

    # initial covariance per timestep (process noise)
    Q = np.identity(2, dtype=float)*1e-4/50

    # vision measurement covariance
    R = np.identity(2, dtype=float)*2e-6

    CUBE_HEIGHT = 0.3

    VISION_TOLERANCE = 0.4  # m

    def on_enable(self):
        self.reset()

    def reset(self):
        self.kalman = Kalman(x_hat=self.chassis.position, P=self.P_init,
                             residual_z=angle_residual)
        self.use_vision = False
        self.cube = np.zeros((2, 1))
        self.last_position = self.chassis.position
        self.odometry_deque = deque([], maxlen=50)
        self.imu_deque = deque([], maxlen=50)
        self.last_vision_tm = self.vision.time
        self.disable_update()

    def predict(self, timesteps_ago=None):
        if timesteps_ago is None:
            self.odometry_deque.append(self.chassis.position)
            # theta_imu = (self.imu.getAngle() + self.chassis.odometry_z_vel+(1/50)/2)
            theta_imu = self.imu.getAngle()
            self.imu_deque.append(theta_imu)
            timesteps_ago = 0
        pos = self.odometry_deque[-timesteps_ago-1]
        last_pos = pos
        if len(self.odometry_deque) >= 2:
            last_pos = self.odometry_deque[-timesteps_ago-2]
        position_delta = pos - last_pos
        self.kalman.predict(np.identity(2), position_delta, np.identity(2),
                            Q=self.Q)

    def update(self, steps_since_vision, vision_data):
        # print("update")
        if not self.imu_deque:
            return
        imu_rollback = min(steps_since_vision, len(self.imu_deque)-1)
        theta_imu = self.imu_deque[-imu_rollback-1]

        def observation(state):
            cube_from_robot = self.cube-state
            field_azimuth = math.atan2(cube_from_robot[1, 0], cube_from_robot[0, 0])
            azimuth = constrain_angle(field_azimuth - theta_imu)
            zenith = math.atan2(np.linalg.norm(cube_from_robot),
                                -self.vision.CAMERA_HEIGHT+self.CUBE_HEIGHT/2)
            # print("state obs az %s zen %s" % (azimuth, zenith))
            return np.array([[azimuth], [zenith]], dtype=float)

        measurement = np.array([[vision_data[0]], [vision_data[1]]], dtype=float)
        self.kalman.unscented_update(measurement, z_dim=2, R=self.R, h=observation)

    def execute(self):
        self.chassis.update_odometry()
        self.predict()
        vision_tm = self.vision.time
        vision_data = self.vision.data
        if (not self.last_vision_tm == vision_tm
                and len(vision_data) > 1):
            since_vision_recieved = time.monotonic()-vision_tm
            since_vision = since_vision_recieved + vision_data[-1]
            steps_since_vision = int(since_vision * 50)
            cube_vision_pos = self.vision_to_field(vision_data[0], vision_data[1])
            if (steps_since_vision > len(self.odometry_deque)-1 or
               not np.allclose(cube_vision_pos, self.cube, rtol=0, atol=self.VISION_TOLERANCE)):
                return
            # print("steps since %s" % since_vision)
            self.kalman.roll_back(steps_since_vision)
            self.update(steps_since_vision, vision_data)
            for i in range(steps_since_vision, 0, -1):
                self.predict(timesteps_ago=steps_since_vision-1)
            self.last_vision_tm = vision_tm
        # print("Chassis odom %s" % self.chassis.position)
        # print("Filter %s" % self.position)

    def set_cube_pos(self, pos):
        self.cube = np.array(pos).reshape((2, 1))

    def enable_update(self):
        self.update_enabled = True

    def disable_update(self):
        self.update_enabled = False

    def vision_to_field(self, azimuth, zenith):
        cube_z = -self.vision.CAMERA_HEIGHT + 0.15

        x = -cube_z*math.tan(math.pi-zenith)*math.cos(azimuth)
        y = x*math.tan(azimuth)
        oriented_x = x * math.cos(self.imu.getAngle()) - y * math.sin(self.imu.getAngle())
        oriented_y = x * math.sin(self.imu.getAngle()) + y * math.cos(self.imu.getAngle())

        cube_rel = np.array([[oriented_x], [oriented_y]])
        cube_derived_pos = cube_rel + self.position
        return cube_derived_pos

    @property
    def position(self):
        return np.reshape(self.kalman.x_hat, 2)


def angle_residual(a, b):
    subtraction = a-b
    return np.arctan2(np.sin(subtraction), np.cos(subtraction))
