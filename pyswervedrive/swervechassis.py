import math
import numpy as np
from wpilib import PIDController
from wpilib.interfaces import PIDOutput
from utilities.bno055 import BNO055
from pyswervedrive.swervemodule import SwerveModule
from wpilib import SmartDashboard
from networktables import NetworkTables
from utilities.functions import constrain_angle


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
        self.field_oriented = True
        self.hold_heading = True
        self.momentum = False

    def setup(self):
        # Heading PID controller
        self.heading_pid_out = ChassisPIDOutput()
        self.heading_pid = PIDController(Kp=6.0, Ki=0.0, Kd=0.2,
                                         source=self.bno055.getAngle,
                                         output=self.heading_pid_out,
                                         period=1/50)
        self.heading_pid.setInputRange(-math.pi, math.pi)
        self.heading_pid.setOutputRange(-3, 3)
        self.heading_pid.setContinuous()
        self.heading_pid.enable()
        self.modules = [self.module_a, self.module_b, self.module_c, self.module_d]

        self.odometry_x = 0
        self.odometry_y = 0
        self.odometry_theta = 0
        self.odometry_x_vel = 0
        self.odometry_y_vel = 0
        self.odometry_z_vel = 0

    def set_heading_sp_current(self):
        self.set_heading_sp(self.bno055.getAngle())

    def set_heading_sp(self, setpoint):
        self.heading_pid.setSetpoint(setpoint)
        self.heading_pid.enable()
        self.momentum = False

    def heading_hold_on(self):
        self.set_heading_sp_current()
        self.heading_pid.reset()
        self.hold_heading = True

    def heading_hold_off(self):
        self.heading_pid.disable()
        self.hold_heading = False

    def on_enable(self):
        self.bno055.resetHeading()
        self.heading_hold_on()

        # matrix which translates column vector of [x, y] in robot frame of
        # reference to module [x, y] movement
        self.A_matrix = np.array([
            [1, 0],
            [0, 1],
            [1, 0],
            [0, 1],
            [1, 0],
            [0, 1],
            [1, 0],
            [0, 1]
            ])

        # figure out the contribution of the robot's overall rotation about the
        # z axis to each module's movement, and encode that information in a
        # column vector
        self.z_axis_adjustment = np.zeros((8, 1))
        for i, module in enumerate(self.modules):
            module_dist = math.hypot(module.x_pos, module.y_pos)
            module_angle = math.atan2(module.y_pos, module.x_pos)
            self.z_axis_adjustment[i*2, 0] = -module_dist*math.sin(module_angle)
            self.z_axis_adjustment[i*2+1, 0] = module_dist*math.cos(module_angle)

        for module in self.modules:
            module.reset_steer_setpoint()

        self.last_heading = self.bno055.getAngle()

    def execute(self):

        pid_z = 0
        if self.hold_heading:
            if self.momentum and abs(self.bno055.getHeadingRate()) < 0.005:
                self.momentum = False
            if self.vz not in [0.0, None]:
                self.momentum = True
            if self.vz is None:
                self.momentum = False

            if not self.momentum:
                pid_z = self.heading_pid.get()
            else:
                self.set_heading_sp_current()
        input_vz = 0
        if self.vz is not None:
            input_vz = self.vz
        vz = input_vz + pid_z

        for module in self.modules:
            module_dist = math.hypot(module.x_pos, module.y_pos)
            module_angle = math.atan2(module.y_pos, module.x_pos)
            # Calculate the additional vx and vy components for this module
            # required to achieve our desired angular velocity
            vz_x = -module_dist*vz*math.sin(module_angle)
            vz_y = module_dist*vz*math.cos(module_angle)
            # TODO: re enable this and test field-oriented mode
            if self.field_oriented:
                angle = self.bno055.getAngle()
                vx, vy = self.robot_orient(self.vx, self.vy, angle)
            else:
                vx, vy = self.vx, self.vy
            module.set_velocity(vx+vz_x, vy+vz_y)

        odometry_outputs = np.zeros((8, 1))
        velocity_outputs = np.zeros((8, 1))

        heading = self.bno055.getAngle()
        heading_delta = constrain_angle(heading - self.last_heading)
        timestep_average_heading = heading - heading_delta / 2

        for i, module in enumerate(self.modules):
            odometry_x, odometry_y = module.get_cartesian_delta()
            velocity_x, velocity_y = module.get_cartesian_vel()
            odometry_outputs[i*2, 0] = odometry_x
            odometry_outputs[i*2+1, 0] = odometry_y
            velocity_outputs[i*2, 0] = velocity_x
            velocity_outputs[i*2+1, 0] = velocity_y
            module.reset_encoder_delta()

        z_adj_delta = self.z_axis_adjustment * heading_delta
        z_adj_vel = self.z_axis_adjustment * self.bno055.getHeadingRate()

        odometry_outputs = odometry_outputs - z_adj_delta
        velocity_outputs = velocity_outputs - z_adj_vel

        delta_x, delta_y = self.robot_movement_from_odometry(odometry_outputs, timestep_average_heading)
        v_x, v_y = self.robot_movement_from_odometry(velocity_outputs, heading)

        self.odometry_x += delta_x
        self.odometry_y += delta_y
        self.odometry_x_vel = v_x
        self.odometry_y_vel = v_y

        SmartDashboard.putNumber('module_a_speed', self.modules[0].current_speed)
        SmartDashboard.putNumber('module_b_speed', self.modules[1].current_speed)
        SmartDashboard.putNumber('module_c_speed', self.modules[2].current_speed)
        SmartDashboard.putNumber('module_d_speed', self.modules[3].current_speed)
        SmartDashboard.putNumber('module_a_pos', self.modules[0].current_measured_azimuth)
        SmartDashboard.putNumber('module_b_pos', self.modules[1].current_measured_azimuth)
        SmartDashboard.putNumber('module_c_pos', self.modules[2].current_measured_azimuth)
        SmartDashboard.putNumber('module_d_pos', self.modules[3].current_measured_azimuth)
        SmartDashboard.putNumber('odometry_x', self.odometry_x)
        SmartDashboard.putNumber('odometry_y', self.odometry_y)
        SmartDashboard.putNumber('odometry_delta_x', delta_x)
        SmartDashboard.putNumber('odometry_delta_y', delta_y)
        SmartDashboard.putNumber('odometry_x_vel', self.odometry_x_vel)
        SmartDashboard.putNumber('odometry_y_vel', self.odometry_y_vel)
        SmartDashboard.putNumber('imu_heading', heading)
        SmartDashboard.putNumber('heading_delta', heading_delta)
        SmartDashboard.putNumber('average_heading', timestep_average_heading)
        NetworkTables.flush()

        self.last_heading = heading

    def robot_movement_from_odometry(self, odometry_outputs, angle):
        lstsq_ret = np.linalg.lstsq(self.A_matrix, odometry_outputs,
                                    rcond=-1)
        x, y = lstsq_ret[0].reshape(2)
        x_field, y_field = self.field_orient(x, y, angle)
        return x_field, y_field

    def set_velocity_heading(self, vx, vy, heading):
        """Set a translational velocity and a rotational orientation to achieve.

        Args:
            vx: (forward) component of the robot's desired velocity. In m/s.
            vy: (leftward) component of the robot's desired velocity. In m/s.
            heading: the heading the robot is to face.
        """
        self.vx = vx
        self.vy = vy
        self.vz = None
        self.set_heading_sp(heading)

    def set_inputs(self, vx, vy, vz):
        """Set chassis vx, vy, and vz components of inputs.
        Args:
            vx: (forward) component of the robot's desired velocity. In m/s.
            vy: (leftward) component of the robot's desired velocity. In m/s.
            vz: The vz (counter-clockwise rotation) component of the robot's
                desired [angular] velocity. In radians/s.
        """
        self.vx = vx
        self.vy = vy
        self.vz = vz

    def set_field_oriented(self, field_oriented):
        self.field_oriented = field_oriented

    @staticmethod
    def robot_orient(vx, vy, heading):
        """Turn a vx and vy relative to the field into a vx and vy based on the
        robot.

        Args:
            vx: vx to robot orient
            vy: vy to robot orient
            heading: current heading of the robot. In radians CCW from +x axis.
        Returns:
            float: robot oriented vx speed
            float: robot oriented vy speed
        """
        oriented_vx = vx * math.cos(heading) + vy * math.sin(heading)
        oriented_vy = -vx * math.sin(heading) + vy * math.cos(heading)
        return oriented_vx, oriented_vy

    @staticmethod
    def field_orient(vx, vy, heading):
        """Turn a vx and vy relative to the robot into a vx and vy based on the
        field.

        Args:
            vx: vx to field orient
            vy: vy to field orient
            heading: current heading of the robot. In radians CCW from +x axis.
        Returns:
            float: field oriented vx speed
            float: field oriented vy speed
        """
        oriented_vx = vx * math.cos(heading) - vy * math.sin(heading)
        oriented_vy = vx * math.sin(heading) + vy * math.cos(heading)
        return oriented_vx, oriented_vy


class ChassisPIDOutput(PIDOutput):
    def pidWrite(self, output):
        self.output = output
