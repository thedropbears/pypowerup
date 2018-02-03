import ctre


class Lifter:
    lift_motor: ctre.WPI_TalonSRX
    #  TODO find encoder values, tune height values to include robot height and cube
    COUNTS_PER_REV = 1
    DISTANCE_PER_COUNT = 1  # in m

    SWITCH_HEIGHT = 47.625
    LOWER_SCALE = 121.92
    BALANCED_SCALE = 152.4
    UPPER_SCALE = 182.88
    GROUND_HEIGHT = 0

    THRESHOLD = 100

    def setup(self):
        """This is called after variables are injected by magicbot."""
        self.set_pos = self.GROUND_HEIGHT
        self.default_height = self.UPPER_SCALE

        # motion profile stuff does not work yet in simulation
        return

        self.lift_motor.overrideLimitSwitchesEnable(True)
        #  TODO tune motion profiling
        self.lift_motor.configMotionAcceleration(100, 10)
        self.lift_motor.configMotionCruiseVelocity(775, 10)
        self.lift_motor.configMotionProfileTrajectoryPeriod(1, 10)

    def on_enable(self):
        """This is called whenever the robot transitions to being enabled."""
        self.reset()

    def on_disable(self):
        """This is called whenever the robot transitions to disabled mode."""
        self.stop()

    def execute(self):
        """Run at the end of every control loop iteration."""
        pass

    def stop(self):
        """Stop the lift motor"""
        self.lift_motor.stopMotor()

    def reset(self):
        self.set_pos = self.GROUND_HEIGHT * self.DISTANCE_PER_COUNT
        self.lift_motor.set(mode=self.lift_motor.ControlMode.MotionMagic, value=self.set_pos)

    def move(self):
        """Move lift to pos set on controller"""
        self.set_pos = self.default_height * self.DISTANCE_PER_COUNT
        self.lift_motor.set(mode=self.lift_motor.ControlMode.MotionMagic, value=self.set_pos)

    def move_to_height(self, input_setpos):
        """Move lift to height on encoder

        Args:
            input_setpos (int): Encoder position to move lift to in cm.
        """
        self.set_pos = input_setpos * self.DISTANCE_PER_COUNT
        self.lift_motor.set(mode=self.lift_motor.ControlMode.MotionMagic, value=self.set_pos)

    def move_to_encoder_pos(self, input_setpos):
        """Move lift to position on encoder

        Args:
            input_setpos (int): Encoder position to move lift to in encoder counts
        """
        self.set_pos = input_setpos
        self.lift_motor.set(mode=self.lift_motor.ControlMode.MotionMagic, value=self.set_pos)

    def get_pos(self):
        """Returns encoder position on lift

       Returns:
            int: The location of the lift
        """
        return self.lift_motor.getSelectedSensorPosition(0)

    def at_pos(self):
        """Finds if cube location is at setops and within threshold

        Args:
            pos (int): The target encoder position

        Returns:
            bool: If the encoder is at the pos
        """
        current_pos = self.get_pos()
        return current_pos - self.THRESHOLD <= self.set_pos <= current_pos + self.THRESHOLD
