from ctre import WPI_TalonSRX
import wpilib


class Intake:

    limit_switch = WPI_TalonSRX("Channel")
    intake_motor1 = WPI_TalonSRX("Channel")
    """This controls the front section of the intake mechanism,
    This controls two motors."""
    intake_motor2 = WPI_TalonSRX("Channel")
    """This controls the back section of the intake mechanism,
    this controls two motors."""
    arm = wpilib.solenoid(0)

    shoot = wpilib.solenoid(1)

    def setup(self):
        """This is called after variables are injected by magicbot."""
        pass

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
        """Run when the limit switch is pressed and when the current
        output is above a threshold, which stops the motors."""
        if (self.intake_motor.getOutputCurrent() > 3 and
                self.limit_switch.getLimitSwitchState() == "something"):
            return True
        else:
            return False

    def cube_outside(self):
        """Run when a button is pushed on the joystick. Makes the
        wheels back drive."""
        if (self.intake_motor.getOutputCurrent() < 3 and
                self.limit_switch.getLimitSwitchState() == "something"):
            return True
        else:
            return False

    def intake_arm(self, value):
        """Turns intake arm on or off"""
        self.arm.set(value)

    def intake_push(self, value):
        """Turns the pushing pneumatic on or off"""
        self.shoot.set(value)
