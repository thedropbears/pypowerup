from ctre import WPI_TalonSRX


class Lifter:

    def setup(self):
        """This is called after variables are injected by magicbot."""
        self.switch_height = None

        self.lower_scale = None
        self.balanced_scale = None
        self.upper_scale = None

        self.exchange_height = None
        self.ground_height = None

        self.threshold = 100

        self.motor: WPI_TalonSRX
        self.mode = WPI_TalonSRX.ControlMode.Position

        self.motor.config_kP(0)
        self.motor.config_kI(0)
        self.motor.config_kD(0)

        self.set_pos = None

    def on_enable(self):
        """This is called whenever the robot transitions to being enabled."""
        pass

    def on_disable(self):
        """This is called whenever the robot transitions to disabled mode."""
        pass

    def execute(self):
        """Run at the end of every control loop iteration."""
        pass

    def move_switch(self):
        """Move the lift to the height of the switch
        """
        self.move_to(self.switch_height)

    def move_exchange(self):
        """ Move the lift to the height of the exchange
        """
        self.move_to(self.exchange_height)
    
    def reset_pos(self):
        """Move to ground height"""
        self.move_to(self.ground_height)

    def stop(self):
        """Stop the lift motor"""
        self.motor.stopMotor()

    def move_lower_scale(self):
        """Move the lift to the lowest height of the scale."""
        self.set_pos(self.lower_scale)

    def move_balanced_scale(self):
        """Move the lift to the balanced height of the scale."""
        self.set_pos(self.balanced_scale)

    def move_upper_scale(self):
        """Move the lift to the upper height of the scale."""
        self.set_pos(self.upper_scale)

    def move_to(self, input_setpos):
        """Run pid loop to position on encoder

        Args:
            setpos (int): Encoder position to move lift to.
        """
        self.set_pos = input_setpos
        self.motor.set(mode=self.mode, value=self.setpos)

    def get_pos(self):
        """Returns encoder position on lift

       Returns:
            int: The location of the lift
        """
        return self.motor.getSelectedSensorPosition(0)

    def at_pos(self):
        """Finds if cube location is within threshold

        Args:
            pos (int): The target encoder position

        Returns:
            bool: If the encoder is at the pos
        """
        current_pos = self.get_pos
        return self.set_pos <= current_pos + self.threshold and self.set_pos >= current_pos - self.threshold
