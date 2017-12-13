from pyswervedrive.swervemodule import SwerveModule
from utilities.bno055 import BNO055
import math

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
        self.field_oriented = True

    def setup(self):
        self.modules = [self.module_a, self.module_b, self.module_c, self.module_d]

    def execute(self):
        for module in self.modules:
            module_dist = math.sqrt(module.cfg.x_pos**2+module.cfg.y_pos**2)
            module_angle = math.atan2(module.cfg.y_pos, module.cfg.x_pos)
            # Calculate the additional vx and vy components for this module
            # required to achieve our desired angular velocity
            vz_x = -module_dist*self.vz*math.sin(module_angle)
            vz_y = module_dist*self.vz*math.cos(module_angle)
            # TODO: re enable this and test field-oriented mode
            # if self.field_oriented:
            #     vx, vy = self.field_orient(self.vx, self.vy, self.bno055.getAngle())
            module.set_velocity(self.vx+vz_x, self.vy+vz_y)

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
