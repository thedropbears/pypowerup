import math

import wpilib
from robotpy_ext.common_drivers.navx.ahrs import AHRS
from robotpy_ext.common_drivers.navx.registerio_spi import RegisterIO_SPI

from utilities.functions import constrain_angle


class NavX(AHRS):

    def getAngle(self):
        raw = super().getAngle()
        return constrain_angle(-math.radians(raw))

    def resetHeading(self):
        self.reset()

    def getHeadingRate(self):
        return math.radians(-self.getRate())

    # fix broken create_spi
    @classmethod
    def create_spi(cls, port=wpilib.SPI.Port.kMXP, spi_bitrate=None, update_rate_hz=None):
        """Constructs the AHRS class using SPI communication, overriding the
        default update rate with a custom rate which may be from 4 to 100,
        representing the number of updates per second sent by the sensor.

        This constructor allows the specification of a custom SPI bitrate, in bits/second.

        .. note:: Increasing the update rate may increase the CPU utilization.

        :param port: SPI Port to use
        :type port: :class:`.SPI.Port`
        :param spi_bitrate: SPI bitrate (Maximum:  2,000,000)
        :param update_rate_hz: Custom Update Rate (Hz)
        """

        io = RegisterIO_SPI(port, spi_bitrate)
        return cls(io, update_rate_hz)

    def getYaw(self):
        return -math.radians(super().getYaw())
