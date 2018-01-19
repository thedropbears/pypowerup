from ctre import CANTalon
from wpilib import DigitalInput


class Intake:

    intake_motor = CANTalon("Channel")
    switch = DigitalInput("Channel")

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

    def intake_rotate(self, value):
        """Turns intake mechanism on."""
        self.intake_motor.set(value)

    def intake_disable(self):
        """Turns intake mechanism off."""
        self.intake_motor.set(0.0)

    def cube_inside(self):
        """Run when the micro switch is pressed and when the current
        output is above a threshold, which stops the motors."""
        if self.intake_motor.getOutputCurrent() > 5 and self.switch.get() == 1:
            return True
        else:
            return False

    def cube_outside(self):
        """Run when a button is pushed on the joystick. Makes the
        wheels back drive."""
        if self.intake_motor.getOutputCurrent() < 5 and self.switch.get() == 0:
            return True
        else:
            return False
