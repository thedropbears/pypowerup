import ctre
import wpilib
import hal


class Lifter:
    motor: ctre.TalonSRX
    centre_switch: wpilib.DigitalInput
    top_switch: wpilib.DigitalInput

    #  TODO find encoder values, tune height values to include robot height and cube
    COUNTS_PER_REV = 4096
    NUM_TEETH = 42
    TEETH_DISTANCE = 0.005
    # double distance for the 2nd stage dolly
    COUNTS_PER_METRE = COUNTS_PER_REV / (NUM_TEETH * TEETH_DISTANCE) / 2

    MOTOR_FREE_SPEED = 5840 / 600  # RPM to rotations/100ms
    GEAR_RATIO = 1 / 14
    FREE_SPEED = int(MOTOR_FREE_SPEED * GEAR_RATIO * COUNTS_PER_REV)  # counts/100ms

    TOP_HEIGHT = 2  # in m
    BOTTOM_HEIGHT = 0
    THRESHOLD = 0.005

    HEIGHT_FROM_FLOOR = 0.17  # height from floor to initial lift pos when reset in m
    CONTAINMENT_SIZE = 0  # height needed for the mechanism to work properly in m

    CUBE_HEIGHT = 0.3

    # heights in m
    UPPER_SCALE = 1.8288 - HEIGHT_FROM_FLOOR + CONTAINMENT_SIZE + CUBE_HEIGHT
    BALANCED_SCALE = 1.524 - HEIGHT_FROM_FLOOR + CONTAINMENT_SIZE + CUBE_HEIGHT
    LOWER_SCALE = 1.2192 - HEIGHT_FROM_FLOOR + CONTAINMENT_SIZE + CUBE_HEIGHT
    # SWITCH = 0.47625 - HEIGHT_FROM_FLOOR + CONTAINMENT_SIZE
    SWITCH = 1.0 - HEIGHT_FROM_FLOOR + CONTAINMENT_SIZE

    UPWARD_ACCELERATION = FREE_SPEED
    DOWNWARD_ACCELERATION = FREE_SPEED*2

    def setup(self):
        """This is called after variables are injected by magicbot."""
        self.set_pos = None

        self.motor.setQuadraturePosition(0, timeoutMs=10)
        self.motor.setInverted(True)
        self.motor.setNeutralMode(ctre.NeutralMode.Brake)

        self.motor.overrideLimitSwitchesEnable(False)
        self.motor.configReverseLimitSwitchSource(ctre.TalonSRX.LimitSwitchSource.FeedbackConnector, ctre.TalonSRX.LimitSwitchNormal.NormallyOpen, deviceID=0, timeoutMs=10)
        self.motor.configForwardLimitSwitchSource(ctre.TalonSRX.LimitSwitchSource.Deactivated, ctre.TalonSRX.LimitSwitchNormal.Disabled, deviceID=0, timeoutMs=10)

        self.motor.overrideSoftLimitsEnable(True)
        self.motor.configForwardSoftLimitEnable(True, timeoutMs=10)
        self.motor.configForwardSoftLimitThreshold(self.metres_to_counts(self.TOP_HEIGHT), timeoutMs=10)
        self.motor.configReverseSoftLimitEnable(False, timeoutMs=10)

        if not hal.isSimulation():
            self.motor.configSetParameter(ctre.TalonSRX.ParamEnum.eClearPositionOnLimitR, 1, 0, 0, timeoutMs=10)

        self.motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, timeoutMs=10)

        # TODO tune motion profiling
        self.motor.selectProfileSlot(0, 0)
        self.motor.config_kF(0, 1023/self.FREE_SPEED, timeoutMs=10)
        self.motor.config_kP(0, 2, timeoutMs=10)
        self.motor.config_kI(0, 0, timeoutMs=10)
        self.motor.config_kD(0, 1, timeoutMs=10)

        self.motor.configMotionAcceleration(self.UPWARD_ACCELERATION, timeoutMs=10)
        self.motor.configMotionCruiseVelocity(int(self.FREE_SPEED), timeoutMs=10)

    def on_disable(self):
        """This is called whenever the robot transitions to disabled mode."""
        self.stop()

    def execute(self):
        """Run at the end of every control loop iteration."""

    @classmethod
    def metres_to_counts(cls, metres):
        return int(metres * cls.COUNTS_PER_METRE)

    def stop(self):
        """Stop the lift motor"""
        self.set_pos = None
        self.motor.neutralOutput()

    def reset(self):
        self.move(self.BOTTOM_HEIGHT)

    def move(self, input_setpos):
        """Move lift to height on encoder

        Args:
            input_setpos (int): Encoder position to move lift to in m.
        """
        self.set_pos = input_setpos
        if self.set_pos - self.get_pos() < 0:
            self.motor.configMotionAcceleration(self.DOWNWARD_ACCELERATION, timeoutMs=0)
        else:
            self.motor.configMotionAcceleration(self.UPWARD_ACCELERATION, timeoutMs=0)

        self.motor.set(ctre.ControlMode.MotionMagic, self.metres_to_counts(self.set_pos))

    def get_pos(self) -> float:
        """Get the current height of the lift."""
        return self.motor.getSelectedSensorPosition(0) / self.COUNTS_PER_METRE

    def get_velocity(self) -> float:
        """Get the current velocity of the lift."""
        return self.motor.getSelectedSensorVelocity(0) / self.COUNTS_PER_METRE

    def get_current(self) -> float:
        return self.motor.getOutputCurrent()

    def manual_down(self):
        self.motor.set(ctre.ControlMode.PercentOutput, -0.5)

    def switch_pressed(self):
        return self.motor.isRevLimitSwitchClosed()

    def at_pos(self):
        """Finds if cube location is at setops and within threshold

        Returns:
            bool: If the encoder is at the pos
        """
        if self.set_pos is None:
            return False
        return abs(self.set_pos - self.get_pos()) < self.THRESHOLD
