"""This is the code to read and write from the BNO055 IMU that we use.

An IMU, or Inertial Measurement Unit, is a device that takes data from
3 accelerometers and 3 gyros (for the 3 different axis) and converts it
into your orientation in 3 dimensions. We use this in order to get our
heading on the field, which is useful for *field orienting* the robot
as well as holding our heading.
"""

import enum
import math
import time

import hal
import wpilib


class BNO055(wpilib.GyroBase):
    """Class to read Euler values in radians from a BNO055 over I2C."""

    # i2c addresses
    # a is when the com3 in input is low
    # b is when it is high
    ADDRESS_A = 0x28
    ADDRESS_B = 0x29

    class Register(enum.IntEnum):
        """Register addresses."""

        CHIP_ID = 0x00
        ACCEL_REV_ID = 0x01
        MAG_REV_ID = 0x02
        GYRO_REV_ID = 0x03
        SW_REV_ID_LSB = 0x04
        SW_REV_ID_MSB = 0x05
        BL_REV_ID = 0x06

        PAGE_ID = 0x07

        ACCEL_DATA_X_LSB = 0x08
        ACCEL_DATA_X_MSB = 0x09
        ACCEL_DATA_Y_LSB = 0x0A
        ACCEL_DATA_Y_MSB = 0x0B
        ACCEL_DATA_Z_LSB = 0x0C
        ACCEL_DATA_Z_MSB = 0x0D

        MAG_DATA_X_LSB = 0x0E
        MAG_DATA_X_MSB = 0x0F
        MAG_DATA_Y_LSB = 0x10
        MAG_DATA_Y_MSB = 0x11
        MAG_DATA_Z_LSB = 0x12
        MAG_DATA_Z_MSB = 0x13

        GYRO_DATA_X_LSB = 0x14
        GYRO_DATA_X_MSB = 0x15
        GYRO_DATA_Y_LSB = 0x16
        GYRO_DATA_Y_MSB = 0x17
        GYRO_DATA_Z_LSB = 0x18
        GYRO_DATA_Z_MSB = 0x19

        EULER_H_LSB = 0x1A  # euler heading, least significant bit
        EULER_H_MSB = 0x1B  # euler heading, most significant bit
        EULER_P_LSB = 0x1C  # euler pitch, least significant bit
        EULER_P_MSB = 0x1D  # euler pitch, most significant bit
        EULER_R_LSB = 0x1E  # euler roll, least significant bit
        EULER_R_MSB = 0x1F  # euler roll, most significant bit

        QUATERNION_DATA_W_LSB = 0x20
        QUATERNION_DATA_W_MSB = 0x21
        QUATERNION_DATA_X_LSB = 0x22
        QUATERNION_DATA_X_MSB = 0x23
        QUATERNION_DATA_Y_LSB = 0x24
        QUATERNION_DATA_Y_MSB = 0x25
        QUATERNION_DATA_Z_LSB = 0x26
        QUATERNION_DATA_Z_MSB = 0x27

        LINEAR_ACCEL_DATA_X_LSB = 0x28
        LINEAR_ACCEL_DATA_X_MSB = 0x29
        LINEAR_ACCEL_DATA_Y_LSB = 0x2A
        LINEAR_ACCEL_DATA_Y_MSB = 0x2B
        LINEAR_ACCEL_DATA_Z_LSB = 0x2C
        LINEAR_ACCEL_DATA_Z_MSB = 0x2D

        GRAVITY_DATA_X_LSB = 0x2E
        GRAVITY_DATA_X_MSB = 0x2F
        GRAVITY_DATA_Y_LSB = 0x30
        GRAVITY_DATA_Y_MSB = 0x31
        GRAVITY_DATA_Z_LSB = 0x32
        GRAVITY_DATA_Z_MSB = 0x33

        TEMP = 0x34

        CALIB_STAT = 0x35
        SELFTEST_RESULT = 0x36
        INTR_STAT = 0x37

        SYS_CLK_STAT = 0x38
        SYS_STAT = 0x39
        SYS_ERR = 0x3A

        UNIT_SEL = 0x3B  # address used to select the unit outputs

        DATA_SELECT = 0x3C

        OPR_MODE = 0x3D  # the address used to select what sensors to use for the gyro outputs

        PWR_MODE = 0x3E

        SYS_TRIGGER = 0x3F
        TEMP_SOURCE = 0x40

        AXIS_MAP_CONFIG = 0x41
        AXIS_MAP_SIGN = 0x42

        SIC_MATRIX_0_LSB = 0x43
        SIC_MATRIX_0_MSB = 0x44
        SIC_MATRIX_1_LSB = 0x45
        SIC_MATRIX_1_MSB = 0x46
        SIC_MATRIX_2_LSB = 0x47
        SIC_MATRIX_2_MSB = 0x48
        SIC_MATRIX_3_LSB = 0x49
        SIC_MATRIX_3_MSB = 0x4A
        SIC_MATRIX_4_LSB = 0x4B
        SIC_MATRIX_4_MSB = 0x4C
        SIC_MATRIX_5_LSB = 0x4D
        SIC_MATRIX_5_MSB = 0x4E
        SIC_MATRIX_6_LSB = 0x4F
        SIC_MATRIX_6_MSB = 0x50
        SIC_MATRIX_7_LSB = 0x51
        SIC_MATRIX_7_MSB = 0x52
        SIC_MATRIX_8_LSB = 0x53
        SIC_MATRIX_8_MSB = 0x54

        ACCEL_OFFSET_X_LSB = 0x55
        ACCEL_OFFSET_X_MSB = 0x56
        ACCEL_OFFSET_Y_LSB = 0x57
        ACCEL_OFFSET_Y_MSB = 0x58
        ACCEL_OFFSET_Z_LSB = 0x59
        ACCEL_OFFSET_Z_MSB = 0x5A

        MAG_OFFSET_X_LSB = 0x5B
        MAG_OFFSET_X_MSB = 0x5C
        MAG_OFFSET_Y_LSB = 0x5D
        MAG_OFFSET_Y_MSB = 0x5E
        MAG_OFFSET_Z_LSB = 0x5F
        MAG_OFFSET_Z_MSB = 0x60

        GYRO_OFFSET_X_LSB = 0x61
        GYRO_OFFSET_X_MSB = 0x62
        GYRO_OFFSET_Y_LSB = 0x63
        GYRO_OFFSET_Y_MSB = 0x64
        GYRO_OFFSET_Z_LSB = 0x65
        GYRO_OFFSET_Z_MSB = 0x66

        ACCEL_RADIUS_LSB = 0x67
        ACCEL_RADIUS_MSB = 0x68
        MAG_RADIUS_LSB = 0x69
        MAG_RADIUS_MSB = 0x6A

    class OperationMode(enum.IntEnum):
        """Operation modes. Changes what sensors it uses and how it fuses them."""

        CONFIG = 0x00
        ACCONLY = 0x01
        MAGONLY = 0x02
        GYRONLY = 0x03
        ACCMAG = 0x04
        ACCGYRO = 0x05
        MAGGYRO = 0x06
        AMG = 0x07
        IMUPLUS = 0x08
        COMPASS = 0x09
        M4G = 0x0A
        NDOF_FMC_OFF = 0x0B
        NDOF = 0x0C

    class PowerMode(enum.IntEnum):
        NORMAL = 0x00
        LOWPOWER = 0x01
        SUSPEND = 0x02

    class AxisMapConfigMode(enum.IntEnum):
        """Axis map configuration modes."""

        P0 = 0x21
        P1 = 0x22
        P2 = 0x23
        P3 = 0x24
        P4 = 0x25
        P5 = 0x26
        P6 = 0x27
        P7 = 0x28

    class AxisMapSign(enum.IntEnum):
        P0 = 0x04
        P1 = 0x00
        P2 = 0x06
        P3 = 0x02
        P4 = 0x03
        P5 = 0x01
        P6 = 0x07
        P7 = 0x05

    # the units that we want and the index in the unit select register that corresponds to it
    UNIT_SEL_ACC_UNIT = 0  # m/s
    UNIT_SEL_ACC_UNIT_INDEX = 0
    UNIT_SEL_GYR_UNIT = 1  # rad/s
    UNIT_SEL_GYR_UNIT_INDEX = 1
    UNIT_SEL_EUL_UNIT = 1  # rad
    UNIT_SEL_EUL_UNIT_INDEX = 2
    UNIT_SEL_TEMP_UNIT = 0  # celcius
    UNIT_SEL_TEMP_UNIT_INDEX = 4
    UNIT_SEL_ORI_UNIT = 1  # android orientation mode, pitch turning clockwise decreases values
    UNIT_SEL_ORI_UNIT_INDEX = 7
    UNIT_SEL_LIST = [
        (UNIT_SEL_ACC_UNIT, UNIT_SEL_ACC_UNIT_INDEX),
        (UNIT_SEL_GYR_UNIT, UNIT_SEL_GYR_UNIT_INDEX),
        (UNIT_SEL_EUL_UNIT, UNIT_SEL_EUL_UNIT_INDEX),
        (UNIT_SEL_TEMP_UNIT, UNIT_SEL_TEMP_UNIT_INDEX),
        (UNIT_SEL_ORI_UNIT, UNIT_SEL_ORI_UNIT_INDEX),
    ]

    def __init__(self, port: wpilib.I2C.Port = None, address: int = None) -> None:
        super().__init__()
        if address is None:
            address = self.ADDRESS_A
        if port is None:
            port = wpilib.I2C.Port.kMXP

        sim_port = None
        if hal.isSimulation():
            from .bno055_sim import BNO055Sim
            sim_port = BNO055Sim()

        self.i2c = wpilib.I2C(port, address, sim_port)

        # set the units that we want
        self.offset = 0.0
        current_units = self.i2c.read(self.Register.UNIT_SEL, 1)[0]
        for wanted, index in self.UNIT_SEL_LIST:
            if wanted == 1:
                current_units = current_units | (1 << index)
            elif wanted == 0:
                current_units = current_units & ~(1 << index)
        self.i2c.write(self.Register.UNIT_SEL, current_units)
        self.setOperationMode(self.OperationMode.IMUPLUS)  # accelerometer and gyro
        self.reverse_axis(False, False, False)
        self.cache_heading()

    def cache_heading(self):
        """Cache the heading for the current timestep to prevent continually polling I2C."""
        self.timestep_cached_heading = self.getRawHeading()
        self.timestep_cached_heading_tm = time.monotonic()

    def reverse_axis(self, x: bool, y: bool, z: bool) -> None:
        """Reverse the specified axis directions."""
        current_directions = self.i2c.read(self.Register.AXIS_MAP_SIGN, 1)[0]
        if x:
            current_directions |= (1 << 2)
        else:
            current_directions &= ~(1 << 2)
        if y:
            current_directions |= (1 << 1)
        else:
            current_directions &= ~(1 << 1)
        if z:
            current_directions |= (1 << 0)
        else:
            current_directions &= ~(1 << 0)
        self.i2c.write(self.Register.AXIS_MAP_SIGN, current_directions)

    def setOperationMode(self, mode: OperationMode) -> None:
        mode = self.OperationMode(mode)
        self.i2c.write(self.Register.OPR_MODE, mode.value)

    def getAngle(self) -> float:
        """Function called by the GyroBase's PID Source to get the current measurement."""
        return self.getHeading()

    def getAngles(self) -> tuple:
        """Return the (heading, pitch, roll) of the gyro."""
        return self.getHeading(), self.getPitch(), self.getRoll()

    def getHeading(self) -> float:
        if time.monotonic() - self.timestep_cached_heading_tm > 1/100:
            self.cache_heading()
        angle = self.timestep_cached_heading - self.offset
        return math.atan2(math.sin(angle), math.cos(angle))

    def getHeadingTime(self) -> float:
        """Get the time that the heading as returned by getHeading was cached at."""
        return self.timestep_cached_heading_tm

    def getRawHeading(self) -> float:
        return -self.getEuler(self.Register.EULER_H_LSB)

    def getPitch(self) -> float:
        return self.getEuler(self.Register.EULER_P_LSB)

    def getRoll(self) -> float:
        return self.getEuler(self.Register.EULER_R_LSB)

    def getEuler(self, start_register: int) -> float:
        euler_bytes = self.i2c.read(start_register, 2)
        euler_int = int.from_bytes(euler_bytes, 'little', signed=True)
        return euler_int / 900.0

    def getHeadingRate(self) -> float:
        return -self.getEuler(self.Register.GYRO_DATA_Z_LSB)

    def resetHeading(self, heading: float = 0) -> None:
        self.offset = self.getRawHeading() - heading
