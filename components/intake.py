from ctre import WPI_TalonSRX
import wpilib
from robotpy_ext.common_drivers.distance_sensors import SharpIRGP2Y0A41SK0F


class Intake:
    intake_left: WPI_TalonSRX
    intake_right: WPI_TalonSRX
    clamp_arm: wpilib.Solenoid
    intake_kicker: wpilib.Solenoid
    extension_arms: wpilib.Solenoid
    infrared: SharpIRGP2Y0A41SK0F
    cube_switch: wpilib.DigitalInput

    def setup(self):
        """This is called after variables are injected by magicbot."""
        self.intake_right.follow(self.intake_left)
        self.intake_right.setInverted(True)
        self.motor_on = 0
        self.clamp_on = False
        self.push_on = False
        self.extension_on = False
        self.arms_out = False

    def on_enable(self):
        """This is called whenever the robot transitions to being enabled."""
        pass

    def on_disable(self):
        """This is called whenever the robot transitions to disabled mode."""
        pass

    def execute(self):
        """Run at the end of every control loop iteration."""
        if self.motor_on == 1:
            self.intake_left.set(1)
        elif self.motor_on == -1:
            self.intake_left.set(-1)
        else:
            self.intake_left.stopMotor()

        if self.clamp_on:
            self.clamp_arm.set(True)
        else:
            self.clamp_arm.set(False)

        if self.push_on:
            self.intake_kicker.set(True)
        else:
            self.intake_kicker.set(False)

        if self.extension_on:
            self.extension_arms.set(True)
        else:
            self.extension_arms.set(False)

    def rotate(self, value):
        """Turns intake mechanism on."""
        self.motor_on = value

    def intake_clamp(self, value):
        """Turns intake arm on or off"""
        self.clamp_on = value

    def intake_push(self, value):
        """Turns the pushing pneumatic on or off"""
        self.push_on = value

    def extension(self, value):
        """Turns both pneumatic extensions on or off"""
        self.extension_on = value

    def infrared_distance(self):
        """Gets the distance of the infrared sensor in mm"""
        self.cube_distance = self.infrared.getDistance()
        self.cube_distance * 10

    def cube_inside(self):
        """Run when the limit switch is pressed and when the current
        output is above a threshold, which stops the motors."""
        # if not self.cube_switch.get():
        # print("limit switch pressed")
        # return True
        # if 100 <= self.cube_distance <= 150:
        # return True
        # return False

    def contacting_cube(self):
        """Returns True of the current output of the motor is above 3"""
        if self.intake_left.getOutputCurrent() >= 5:
            return True
        else:
            return False
