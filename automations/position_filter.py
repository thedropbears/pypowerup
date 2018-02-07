from components.vision import Vision
from pyswervedrive.swervechassis import SwerveChassis
from utilities.bno055 import BNO055
from utilities.functions import constrain_angle
from utilities.kalman import Kalman

from collections import deque
import math
import numpy as np
import time


class PositionFilter:

    bno055: BNO055
    chassis: SwerveChassis
    vision: Vision

    # initial covariance of the system
    P_init = np.identity(2, dtype=float)*1e-4

    # initial covariance per timestep (process noise)
    Q = np.identity(2, dtype=float)*1e-4/50

    # vision measurement covariance
    R = np.identity(2, dtype=float)*2e-6

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
        self.vision.current_cube()
        self.last_vision_tm = self.vision.recieved_time

    def predict(self, timesteps_ago=None):
        if timesteps_ago is None:
            self.odometry_deque.append(self.chassis.position)
            # theta_imu = (self.bno055.getAngle() + self.chassis.odometry_z_vel+(1/50)/2)
            theta_imu = self.bno055.getAngle()
            self.imu_deque.append(theta_imu)
            timesteps_ago = 0
        pos = self.odometry_deque[-timesteps_ago-1]
        last_pos = pos
        if len(self.odometry_deque) >= 2:
            last_pos = self.odometry_deque[-timesteps_ago-2]
        position_delta = pos - last_pos
        self.kalman.predict(np.identity(2), position_delta, np.identity(2),
                            Q=self.Q)

    def update(self, steps_since_vision):
        print("update")
        imu_rollback = min(steps_since_vision, len(self.imu_deque))
        theta_imu = self.imu_deque[-imu_rollback-1]

        def observation(state):
            cube_from_robot = self.cube-state
            field_azimuth = math.atan2(cube_from_robot[1, 0], cube_from_robot[0, 0])
            azimuth = constrain_angle(field_azimuth - theta_imu)
            zenith = math.atan2(np.linalg.norm(cube_from_robot), -self.vision.CAMERA_Z_POS+0.15)
            print("state obs az %s zen %s" % (azimuth, zenith))
            return np.array([[azimuth], [zenith]], dtype=float)

        azimuth_obs, zenith_obs, vision_recieved = self.vision.current_cube()
        measurement = np.array([[azimuth_obs], [zenith_obs]], dtype=float)
        self.kalman.unscented_update(measurement, z_dim=2, R=self.R, h=observation)

    def execute(self):
        self.chassis.update_odometry()
        self.predict()
        self.vision.current_cube()
        if (not self.last_vision_tm == self.vision.recieved_time
                and self.vision.seeing_cube):
            since_vision_recieved = time.monotonic()-self.vision.recieved_time
            since_vision = since_vision_recieved + self.vision.vision[-1]
            steps_since_vision = int(since_vision * 50)
            print("steps since %s" % since_vision)
            self.kalman.roll_back(steps_since_vision)
            self.update(steps_since_vision)
            for i in range(steps_since_vision, 0, -1):
                self.predict(timesteps_ago=steps_since_vision-1)
        print("Chassis odom %s" % self.chassis.position)
        print("Filter %s" % self.position)

    @property
    def position(self):
        return np.reshape(self.kalman.x_hat, 2)


def angle_residual(a, b):
    subtraction = a-b
    return np.arctan2(np.sin(subtraction), np.cos(subtraction))
