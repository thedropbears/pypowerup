from pyswervedrive.swervemodule import SwerveModule
from utilities.bno055 import BNO055
import numpy as np
import math

import hal


class SwerveChassis:

    bno055: BNO055
    module_a: SwerveModule
    module_b: SwerveModule
    module_c: SwerveModule
    module_d: SwerveModule

    def __init__(self):
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.field_oriented = False

    def setup(self):
        self.modules = [self.module_a, self.module_b, self.module_c, self.module_d]

    def on_enable(self):
        # matrix which translates column vector of [x, y, z] in robot frame of
        # reference to module [x, y] movement
        self.A_matrix = np.array([
            [1, 0, -1],
            [0, 1,  1],
            [1, 0, -1],
            [0, 1, -1],
            [1, 0,  1],
            [0, 1, -1],
            [1, 0,  1],
            [0, 1,  1]
            ])
        # figure out the contribution of the robot's overall rotation about the
        # z axis to each module's movement, and encode that information in our
        # matrix
        for i, module in enumerate(self.modules):
            module_dist = math.hypot(module.x_pos,
                                     module.y_pos)
            z_comp = module_dist/2
            # third column in A matrix already encodes direction of robot's
            # vz index upon the module's axis, just need to multiply to
            # encode magnitude
            self.A_matrix[i*2, 2] = z_comp * self.A_matrix[i*2, 2]
            self.A_matrix[i*2+1, 2] = z_comp * self.A_matrix[i*2+1, 2]

        self.odometry_x = 0
        self.odometry_y = 0
        self.odometry_theta = 0

        for module in self.modules:
            module.reset_steer_setpoint()

    def execute(self):
        for module in self.modules:
            module_dist = math.hypot(module.x_pos, module.y_pos)
            module_angle = math.atan2(module.y_pos, module.x_pos)
            # Calculate the additional vx and vy components for this module
            # required to achieve our desired angular velocity
            vz_x = -module_dist*self.vz*math.sin(module_angle)
            vz_y = module_dist*self.vz*math.cos(module_angle)
            # TODO: re enable this and test field-oriented mode
            if self.field_oriented:
                if hal.isSimulation():
                    from hal_impl.data import hal_data
                    angle = math.radians(-hal_data['robot']['bno055'])
                else:
                    angle = self.bno055.getAngle()
                vx, vy = self.field_orient(self.vx, self.vy, angle)
            else:
                vx, vy = self.vx, self.vy
            module.set_velocity(vx+vz_x, vy+vz_y)

        odometry_outputs = np.zeros((8, 1))
        velocity_outputs = np.zeros((8, 1))
        for i, module in enumerate(self.modules):
            odometry_x, odometry_y = module.get_cartesian_delta()
            velocity_x, velocity_y = module.get_cartesian_vel()
            odometry_outputs[i*2, 0] = odometry_x
            odometry_outputs[i*2+1, 0] = odometry_y
            velocity_outputs[i*2, 0] = velocity_x
            velocity_outputs[i*2+1, 0] = velocity_y
            module.reset_encoder_delta()

        delta_x, delta_y, delta_theta = self.robot_movement_from_odometry(
                                            odometry_outputs)
        v_x, v_y, v_z = self.robot_movement_from_odometry(velocity_outputs)

        self.odometry_x += delta_x
        self.odometry_y += delta_y
        self.odometry_theta += delta_theta
        self.odometry_x_vel = v_x
        self.odometry_y_vel = v_y
        self.odometry_z_vel = v_z

    def robot_movement_from_odometry(self, odometry_outputs):
        lstsq_ret = np.linalg.lstsq(self.A_matrix, odometry_outputs,
                                    rcond=-1)
        x, y, theta = lstsq_ret[0].reshape(3)
        angle = self.bno055.getAngle()
        x_field, y_field = self.field_orient(x, y, angle)
        return x_field, y_field, theta

    def set_inputs(self, vx, vy, vz):
        """Set chassis vx, vy, and vz components of inputs.
        :param vx: The vx (forward) component of the robot's desired velocity. In m/s.
        :param vy: The vy (leftward) component of the robot's desired velocity. In m/s.
        :param vz: The vz (counter-clockwise rotation) component of the robot's
        desired [angular] velocity. In radians/s."""
        self.vx = vx
        self.vy = vy
        self.vz = vz

    def set_field_oriented(self, field_oriented):
        self.field_oriented = field_oriented

    @staticmethod
    def field_orient(vx, vy, heading):
        oriented_vx = vx * math.cos(heading) + vy * math.sin(heading)
        oriented_vy = -vx * math.sin(heading) + vy * math.cos(heading)
        return oriented_vx, oriented_vy
