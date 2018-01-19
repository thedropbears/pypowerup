from ctre import CANTalon

class Intake:

    intake_motor = CANTalon

    def setup(self):
        """This is called after variables are injected by magicbot."""
        self.PWMTalonSRX("PWM Channel")
        self.PWMTalonSRX("PWM Channel")

    def on_enable(self):
        """This is called whenever the robot transitions to being enabled."""
        pass

    def on_disable(self):
        """This is called whenever the robot transitions to disabled mode."""
        pass

    def execute(self):
        """Run at the end of every control loop iteration."""
        pass
    
    def intake_enable(self, value):
        """Turns intake mechanism on."""
        self.intake_motor.set(value)
    
    def intake_disable(self):
        """Turns intake mechanism off."""
        pass

    def cube_inside(self):
        """Run when the micro switch is pressed and when the current output is above a threshold, which stops the motors."""
        if 
        return self.intake_motor.getOutputCurrent() > 3

    def cube_outside(self):
        """Run when a button is pushed on the joystick. Makes the wheels back drive."""
        pass