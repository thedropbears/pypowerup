import math
from pyswervedrive.swervemodule import SwerveModule


class PhysicsEngine:

    X_WHEELBASE = 0.62
    Y_WHEELBASE = 0.52

    def __init__(self, controller):
        self.controller = controller

        self.drive_counts_per_rev = \
            SwerveModule.CIMCODER_COUNTS_PER_REV*SwerveModule.DRIVE_ENCODER_GEAR_REDUCTION
        self.drive_counts_per_meter = \
            self.drive_counts_per_rev / (math.pi * SwerveModule.WHEEL_DIAMETER)

        # factor by which to scale velocities in m/s to give to our drive talon.
        # 0.1 is because SRX velocities are measured in ticks/100ms
        self.drive_velocity_to_native_units = self.drive_counts_per_meter*0.1

        # for modules [a, b, c, d]. used to iterate over them
        self.module_steer_can_ids = [2, 11, 8, 4]
        self.module_drive_can_ids = [9, 13, 6, 14]
        self.module_steer_offsets = [-2055, -2583, -1665, -286]

        self.controller.add_device_gyro_channel('bno055')

    def initialize(self, hal_data):
        pass

    def update_sim(self, hal_data, now, tm_diff):
        """
        Update pyfrc simulator.
        :param hal_data: Data about motors and other components
        :param now: Current time in ms
        :param tm_diff: Difference between current time and time when last checked
        """

        steer_positions = []
        for can_id, offset in zip(self.module_steer_can_ids, self.module_steer_offsets):
            value = hal_data['CAN'][can_id]['value']
            hal_data['CAN'][can_id]['pulse_width_position'] = value
            position = (hal_data['CAN'][can_id]['pulse_width_position']-offset) / SwerveModule.STEER_COUNTS_PER_RADIAN
            steer_positions.append(position)

        motor_speeds = []
        for i, can_id in enumerate(self.module_drive_can_ids):
            # divide value by 1023 as it gets multiplied by 1023 in talon code
            speed_sp = hal_data['CAN'][can_id]['value']
            speed = speed_sp / SwerveModule.drive_velocity_to_native_units
            hal_data['CAN'][can_id]['quad_position'] += int(speed_sp*10*tm_diff)
            hal_data['CAN'][can_id]['quad_velocity'] = int(speed_sp)
            motor_speeds.append(speed)

        lf_speed, lr_speed, rr_speed, rf_speed = motor_speeds

        lf_angle, lr_angle, rr_angle, rf_angle = steer_positions
        vx, vy, vw = \
            four_motor_swerve_drivetrain(lr_speed, rr_speed, lf_speed, rf_speed,
                                         lr_angle, rr_angle, lf_angle, rf_angle,
                                         x_wheelbase=self.X_WHEELBASE, y_wheelbase=self.Y_WHEELBASE)
        vx *= 0.3048
        vy *= 0.3048
        vw *= -1
        self.controller.vector_drive(vx, vy, vw, tm_diff)


def four_motor_swerve_drivetrain(lr_speed, rr_speed, lf_speed, rf_speed, lr_angle, rr_angle, lf_angle, rf_angle,
                                 x_wheelbase=2, y_wheelbase=2):
    '''
        Four motors that can be rotated in any direction
        If any motors are inverted, then you will need to multiply that motor's
        value by -1.
        :param lr_speed:   Left rear motor speed. In m/s
        :param rr_speed:   Right rear motor speed. In m/s
        :param lf_speed:   Left front motor speed. In m/s
        :param rf_speed:   Right front motor speed. In m/s
        :param lr_angle:   Left rear motor angle in radians (measured counterclockwise from forward position)
        :param rr_angle:   Right rear motor angle in radians (measured counterclockwise from forward position)
        :param lf_angle:   Left front motor angle in radians (measured counterclockwise from forward position)
        :param rf_angle:   Right front motor angle in radians (measured counterclockwise from forward position)
        :param x_wheelbase: The distance in feet between right and left wheels. In m
        :param y_wheelbase: The distance in feet between forward and rear wheels. In m
        :returns: Speed of robot in x (m/s), Speed of robot in y (m/s),
                  counterclockwise rotation of robot (radians/s)
    '''

    # Calculate wheelbase radius
    wheelbase_radius = math.hypot(x_wheelbase / 2, y_wheelbase / 2)

    # Calculates the Vx and Vy components
    # Sin an Cos inverted because forward is 0 on swerve wheels
    Vx = (math.sin(-lr_angle) * lr_speed) + (math.sin(-rr_angle) * rr_speed) + (math.sin(-lf_angle) * lf_speed) + (math.sin(-rf_angle) * rf_speed)
    Vy = (math.cos(-lr_angle) * lr_speed) + (math.cos(-rr_angle) * rr_speed) + (math.cos(-lf_angle) * lf_speed) + (math.cos(-rf_angle) * rf_speed)

    # Adjusts the angle corresponding to a diameter that is perpendicular to the radius (add or subtract 45deg)
    lr_angle = (-lr_angle + (math.pi / 4)) % (2 * math.pi)
    rr_angle = (-rr_angle - (math.pi / 4)) % (2 * math.pi)
    lf_angle = (-lf_angle - (math.pi / 4)) % (2 * math.pi)
    rf_angle = (-rf_angle + (math.pi / 4)) % (2 * math.pi)

    # Finds the rotational velocity by finding the torque and adding them up
    Vw = wheelbase_radius * -((math.cos(-lr_angle) * lr_speed) + (math.cos(-rr_angle) * -rr_speed) + (math.cos(-lf_angle) * lf_speed) + (math.cos(-rf_angle) * -rf_speed))
    Vw *= -1

    return Vx, Vy, Vw
