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

from .imu import IMU


class BNO055(IMU):
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

        EULER_HEADING_LSB = 0x1A  # yaw
        EULER_HEADING_MSB = 0x1B
        EULER_ROLL_LSB = 0x1C
        EULER_ROLL_MSB = 0x1D
        EULER_PITCH_LSB = 0x1E
        EULER_PITCH_MSB = 0x1F

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

        SYS_CLK_STATUS = 0x38
        SYS_STATUS = 0x39
        SYS_ERR = 0x3A

        UNIT_SEL = 0x3B  # address used to select the unit outputs
        #DATA_SELECT = 0x3C
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

    class Page1Register(enum.IntEnum):
        """Page 1 register addresses. Set Register.PAGE_ID to access these."""

        # Configuration registers
        ACCEL_CONFIG = 0x08
        MAG_CONFIG = 0x09
        GYRO_CONFIG = 0x0A
        GYRO_MODE_CONFIG = 0x0B
        ACCEL_SLEEP_CONFIG = 0x0C
        GYRO_SLEEP_CONFIG = 0x0D
        MAG_SLEEP_CONFIG = 0x0E

        # Interrupt registers
        INT_MASK = 0x0F
        INT = 0x10
        ACCEL_ANY_MOTION_THRES = 0x11
        ACCEL_INTR_SETTINGS = 0x12
        ACCEL_HIGH_G_DURN = 0x13
        ACCEL_HIGH_G_THRES = 0x14
        ACCEL_NO_MOTION_THRES = 0x15
        ACCEL_NO_MOTION_SET = 0x16
        GYRO_INTR_SETING = 0x17
        GYRO_HIGHRATE_X_SET = 0x18
        GYRO_DURN_X_SET = 0x19
        GYRO_HIGHRATE_Y_SET = 0x1A
        GYRO_DURN_Y_SET = 0x1B
        GYRO_HIGHRATE_Z_SET = 0x1C
        GYRO_DURN_Z_SET = 0x1D
        GYRO_ANY_MOTION_THRES = 0x1E
        GYRO_ANY_MOTION_SET = 0x1F

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
        IMU = 0x08  # Inertial Measurement Unit
        COMPASS = 0x09
        M4G = 0x0A  # Magnet for Gyroscope
        NDOF_FMC_OFF = 0x0B
        NDOF = 0x0C

    class PowerMode(enum.IntEnum):
        NORMAL = 0x00
        LOWPOWER = 0x01
        SUSPEND = 0x02

    class AxisMapConfig(enum.IntFlag):
        """Axis remapping configuration flags."""

        X_AXIS_IS_X = 0 << 0
        X_AXIS_IS_Y = 1 << 0
        X_AXIS_IS_Z = 2 << 0

        Y_AXIS_IS_X = 0 << 2
        Y_AXIS_IS_Y = 1 << 2
        Y_AXIS_IS_Z = 2 << 2

        Z_AXIS_IS_X = 0 << 4
        Z_AXIS_IS_Y = 1 << 4
        Z_AXIS_IS_Z = 2 << 4

    class AxisMapSign(enum.IntFlag):
        """Axis sign remapping configuration flags."""

        ALL_POSITIVE = 0
        Z_NEGATIVE = 1 << 0
        Y_NEGATIVE = 1 << 1
        X_NEGATIVE = 1 << 2

    class UnitSel(enum.IntFlag):
        """Flags for Register.UNIT_SEL to select units/data format for data outputs."""

        # Acceleration
        ACC_M_S = 0 << 0  # m/s^2
        ACC_MG = 1 << 0  # mg
        # Gyro angular rate
        GYR_DPS = 0 << 1  # deg/s
        GYR_RPS = 1 << 1  # rad/s
        # Euler angles
        EUL_DEGREES = 0 << 2
        EUL_RADIANS = 1 << 2
        # Temperature
        TEMP_CELSIUS = 0 << 4
        TEMP_FAHRENHEIT = 1 << 4
        # Orientation output format
        ORI_WINDOWS = 0 << 7  # pitch turning clockwise increases values
        ORI_ANDROID = 1 << 7  # pitch turning clockwise decreases values

    def __init__(self, port: wpilib.I2C.Port = None, address: int = None) -> None:
        if address is None:
            address = self.ADDRESS_A
        if port is None:
            port = wpilib.I2C.Port.kMXP

        sim_port = None
        if hal.isSimulation():
            from .bno055_sim import BNO055Sim
            sim_port = BNO055Sim()

        self.i2c = wpilib.I2C(port, address, sim_port)

        self.offset = 0.0
        self.select_units(
            self.UnitSel.ACC_M_S
            | self.UnitSel.GYR_RPS | self.UnitSel.EUL_RADIANS
            | self.UnitSel.TEMP_CELSIUS
            | self.UnitSel.ORI_ANDROID
        )
        self.reverse_axis(False, False, False)
        self.setOperationMode(self.OperationMode.IMU)  # accelerometer and gyro
        self.cache_heading()

    def cache_heading(self):
        """Cache the heading for the current timestep to prevent continually polling I2C."""
        self.timestep_cached_heading = self.getRawHeading()
        self.timestep_cached_heading_tm = time.monotonic()

    def reverse_axis(self, x: bool, y: bool, z: bool) -> None:
        """Reverse the specified axis directions."""
        current_directions = self.AxisMapSign.ALL_POSITIVE
        if x:
            current_directions |= self.AxisMapSign.X_NEGATIVE
        if y:
            current_directions |= self.AxisMapSign.Y_NEGATIVE
        if z:
            current_directions |= self.AxisMapSign.Z_NEGATIVE
        self.i2c.write(self.Register.AXIS_MAP_SIGN, current_directions)

    def setOperationMode(self, mode: OperationMode) -> None:
        mode = self.OperationMode(mode)
        self.i2c.write(self.Register.OPR_MODE, mode.value)

    def select_units(self, units: UnitSel) -> None:
        self.i2c.write(self.Register.UNIT_SEL, units)

    def getAngle(self) -> float:
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
        return -self.getEuler(self.Register.EULER_HEADING_LSB)

    def getPitch(self) -> float:
        return self.getEuler(self.Register.EULER_PITCH_LSB)

    def getRoll(self) -> float:
        return self.getEuler(self.Register.EULER_ROLL_LSB)

    def getEuler(self, start_register: int) -> float:
        euler_bytes = self.i2c.read(start_register, 2)
        euler_int = int.from_bytes(euler_bytes, 'little', signed=True)
        return euler_int / 900.0

    def getHeadingRate(self) -> float:
        return self.getEuler(self.Register.GYRO_DATA_Z_LSB)

    def resetHeading(self, heading: float = 0) -> None:
        self.offset = self.getRawHeading() - heading
