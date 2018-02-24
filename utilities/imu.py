from wpilib.interfaces import PIDSource
from utilities.navx import NavX
from utilities.bno055 import BNO055


class IMU:

    PIDSourceType = PIDSource.PIDSourceType

    def __init__(self, mode):
        self.mode = mode
        if self.mode == 'bno055':
            self.imu = BNO055()
        elif self.mode == 'navx':
            self.imu = NavX()
        self.pidsource = self.PIDSourceType.kDisplacement

    def getAngle(self):
        return self.imu.getAngle()

    def getHeadingRate(self):
        return self.imu.getHeadingRate()

    def resetHeading(self):
        self.imu.resetHeading()

    def pidGet(self):
        if self.pidsource == self.PIDSourceType.kDisplacement:
            return self.getAngle()
        else:
            return self.getHeadingRate()

    def setPIDSourceType(self, pidsourcetype):
        self.pidsource = pidsourcetype

    def getPIDSourceType(self):
        return self.pidsource
