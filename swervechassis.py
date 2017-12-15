from pyswervedrive.swervemodule import SwerveModule
from utilities.bno055 import BNO055
import math

import hal

class SwerveChassis:

    bno055: BNO055
    module_a: SwerveModule
    module_b: SwerveModule
    module_c: SwerveModule
    module_d: SwerveModule

    # multiply both by vz and the number that you need to multiply the vz
    # components by to get them in the appropriate directions

    def __init__(self):
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.field_oriented = False

    def setup(self):
        self.modules = [self.module_a, self.module_b, self.module_c, self.module_d]

    def on_enable(self):
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
