from ctre import WPI_TalonSRX


class Lifter:

    def setup(self):
        """This is called after variables are injected by magicbot."""
        self.switch_height = None
        self.lower_scale = None
        self.balanced_scale = None
        self.upper_scale = None
        self.ground_height = None
        self.motor = WPI_TalonSRX(0)
        self.motor.config_kP(0)
        self.motor.config_kI(0)
        self.motor.config_kD(0)

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
        """Move the lift to the height of the switch."""
        self.set_pos(self.switch_height)

    def move_lower_scale(self):
        """Move the lift to the lowest height of the scale."""
        self.set_pos(self.lower_scale)

    def move_balanced_scale(self):
        """Move the lift to the balanced height of the scale."""
        self.set_pos(self.balanced_scale)

    def move_upper_scale(self):
        """Move the lift to the upper height of the scale."""
        self.set_pos(self.upper_scale)

    def move_ground(self):
        """Move the lift to the ground height of the scale."""
        self.set_pos(self.ground_height)

    def set_pos(self, setpos):
        """Run pid loop to position on encoder

        Args:
            setpos (int): Encoder position to move lift to."""
        mode = WPI_TalonSRX.ControlMode.Position
        self.motor.set(mode=mode, value=setpos)

    def get_pos(self):
        """Returns encoder position on lift

       Returns:
            int: The location of the lift
        """
        return self.motor.getSelectedSensorPosition(0)

    def switchButton(self):
        """Button on keyboard that calls the function move_switch."""
    
    def lower_scaleButton(self):
        """Button on keyboard that calls the function move_lower_scale."""
    
    def balanced_scaleButton(self):
        """Button on keyboard that calls the function move_balanced_scale."""

    def upper_scaleButton(self):
        """Button on keyboard that calls the function move_upper_scale."""