import math

from robotpy_ext.common_drivers.navx.ahrs import AHRS

from .imu import IMU


class NavX(IMU):
    """Wrapper around RobotPy NavX to be compatible with our BNO055 driver."""

    def __init__(self):
        self.ahrs = AHRS.create_spi(update_rate_hz=50)

    def getAngle(self) -> float:
        """Get NavX angle.

        Returns:
            Angle in radians between -pi and +pi.
        """
        raw = self.ahrs.getYaw()
        return -math.radians(raw)

    def resetHeading(self):
        self.ahrs.reset()

    def getHeadingRate(self):
        # multiply by 100 because NavX does not normalise per timestep
        return math.radians(-self.ahrs.getRate()) * 50
