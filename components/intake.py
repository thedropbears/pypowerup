import ctre
import magicbot
import wpilib
from robotpy_ext.common_drivers.distance_sensors import SharpIRGP2Y0A41SK0F


class Intake:
    left_motor: ctre.WPI_TalonSRX
    right_motor: ctre.WPI_TalonSRX
    clamp_arm: wpilib.Solenoid
    intake_kicker: wpilib.Solenoid
    extension_arms: wpilib.Solenoid
    infrared: SharpIRGP2Y0A41SK0F

    arms_out = magicbot.tunable(False, doc='Whether the arms are outside of the starting configuration.')

    def __init__(self):
        self.motor_output = 0
        self.clamp_on = False
        self.push_on = False
        self.extension_on = False

    def setup(self):
        """This is called after variables are injected by magicbot."""
        self.right_motor.follow(self.left_motor)
        self.right_motor.setInverted(True)

    def on_enable(self):
        """This is called whenever the robot transitions to being enabled."""
        pass

    def on_disable(self):
        """This is called whenever the robot transitions to disabled mode."""
        pass

    def execute(self):
        """Run at the end of every control loop iteration."""
        self.left_motor.set(self.motor_output)
        self.clamp_arm.set(self.clamp_on)
        self.intake_kicker.set(self.push_on)
        self.extension_arms.set(self.extension_on)

        # Don't run the motors unless something else commands us to.
        self.motor_output = 0
        # We're not resetting the pneumatics outputs here, as once they
        # have actuated, they stay actuated without drawing power.

    def rotate(self, value: float):
        """Set the output of the intake motors."""
        self.motor_output = value

    def clamp(self, value: bool):
        """Turn the intake clamp on or off."""
        self.clamp_on = value

    def push(self, value: bool):
        """Turn the pushing pneumatic on or off."""
        self.push_on = value

    def extend(self, value: bool):
        """Turn the extension pneumatics on or off."""
        self.extension_on = value

    @magicbot.feedback
    def get_cube_distance(self) -> float:
        """Get the distance of the infrared sensor in m."""
        return self.infrared.getDistance() / 100

    def is_cube_contained(self) -> bool:
        """Check whether a cube is in the containment mechanism."""
        return 0.1 <= self.get_cube_distance() <= 0.15

    def are_wheels_contacting_cube(self) -> bool:
        """Check whether the intake wheels are touching the cube."""
        return self.left_motor.getOutputCurrent() >= 5
