from ctre import WPI_TalonSRX
import wpilib
import math


class Intake:
    intake_left: WPI_TalonSRX
    intake_right: WPI_TalonSRX
    clamp_arm: wpilib.Solenoid
    intake_kicker: wpilib.Solenoid
    extension_arm_left: wpilib.Solenoid
    extension_arm_right: wpilib.Solenoid
    infrared: wpilib.AnalogInput
    cube_switch: wpilib.DigitalInput

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
        self.intake_left.set(value)
        self.intake_right.set(value)

    def intake_disable(self):
        """Turns intake mechanism off."""
        self.intake_left.stopMotor()
        self.intake_right.stopMotor()

    def intake_clamp(self, value):
        """Turns intake arm on or off"""
        self.clamp_arm.set(value)

    def intake_push(self, value):
        """Turns the pushing pneumatic on or off"""
        self.intake_kicker.set(value)

    def extension(self, value):
        """Turns both pneumatic extensions on or off"""
        self.extension_arm_left.set(value)
        self.extension_arm_right.set(value)

    def infraredDistance(self):
        infrared_voltage = self.infrared.getVoltage()
        """Makes sure that the voltage is above 0.00001"""
        voltage = max(infrared_voltage, 0.00001)
        distance = 12.84*math.pow(voltage, -0.9824)
        """This makes sure that the distance is above 4.5 and below 35"""
        self.cube_distance = max(min(distance, 35.0), 4.5)

    def cube_inside(self):
        """Run when the limit switch is pressed and when the current
        output is above a threshold, which stops the motors."""
        if self.cube_switch.get():
            print("Cube is inside")
            return True
        # if 10 <= self.cube_distance <= 15:
        #     return True
        return False

    def button_press(self):
        if self.joystick.getButton(2):
            return True
        return False
