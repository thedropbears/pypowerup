from ctre import WPI_TalonSRX, CANifier
import wpilib


class Intake:
    intake_motor1: WPI_TalonSRX
    intake_motor2: WPI_TalonSRX
    clamp_arm_left: wpilib.Solenoid
    clamp_arm_right: wpilib.Solenoid
    intake_kicker: wpilib.Solenoid
    limit_switch: CANifier

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
                self.limit_switch.getLimitSwitchState()):
            return True

    def cube_outside(self):
        """Run when a button is pushed on the joystick. Makes the
        wheels back drive."""
        if (self.intake_motor.getOutputCurrent() < 3 and
                self.limit_switch.getLimitSwitchState()):
            return True

    def intake_clamp(self, value):
        """Turns intake arm on or off"""
        self.clamp_arm_left.set(value)
        self.clamp_arm_right.set(value)

    def intake_push(self, value):
        """Turns the pushing pneumatic on or off"""
        self.intake_kicker.set(value)

    def switch(self):
        print(self.limit_switch.getPWMInput(4))
