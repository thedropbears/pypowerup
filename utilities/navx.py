import math

import wpilib
from robotpy_ext.common_drivers.navx.ahrs import AHRS
from robotpy_ext.common_drivers.navx.registerio_i2c import RegisterIO_I2C

from utilities.functions import constrain_angle


class NavX(AHRS):

    def getAngle(self):
        raw = super().getAngle()
        return constrain_angle(-math.radians(raw))

    def resetHeading(self):
        self.reset()

    def getHeadingRate(self):
        return math.radians(-self.getRate())

    # fix broken create_i2c
    @classmethod
    def create_i2c(cls, port=wpilib.I2C.Port.kMXP, update_rate_hz=None):
        """Constructs the AHRS class using I2C communication, overriding the
        default update rate with a custom rate which may be from 4 to 100,
        representing the number of updates per second sent by the sensor.

        This constructor should be used if communicating via I2C.

        .. note:: Increasing the update rate may increase the CPU utilization.

        :param port: I2C Port to use
        :type port: :class:`.I2C.Port`
        :param update_rate_hz: Custom Update Rate (Hz)
        """

        io = RegisterIO_I2C(port)
        return cls(io, update_rate_hz)
