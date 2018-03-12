import ctre
import hal
import magicbot
import wpilib
from components.range_finder import RangeFinder


class Intake:
    left_motor: ctre.TalonSRX
    right_motor: ctre.TalonSRX
    clamp_arm: wpilib.Solenoid
    intake_kicker: wpilib.Solenoid
    left_extension: wpilib.Solenoid
    right_extension: wpilib.DoubleSolenoid
    range_finder: RangeFinder

    cube_switch: wpilib.DigitalInput

    arms_out = magicbot.tunable(False, doc='Whether the arms are outside of the starting configuration.')

    def __init__(self):
        self.motor_output = 0
        self.clamp_on = True
        self.push_on = False
        self.extension_on = False
        self.last_clamp_on = None
        self.last_push_on = None
        self.last_extension_on = None
        self.last_motor_output = None

    def setup(self):
        """This is called after variables are injected by magicbot."""
        self.right_motor.follow(self.left_motor)
        self.right_motor.setInverted(True)

    def on_enable(self):
        """This is called whenever the robot transitions to being enabled."""
        self.last_clamp_on = None
        self.last_push_on = None
        self.last_extension_on = None
        self.last_motor_output = None

    def on_disable(self):
        """This is called whenever the robot transitions to disabled mode."""
        pass

    def execute(self):
        """Run at the end of every control loop iteration."""
        if self.motor_output != self.last_motor_output:
            self.left_motor.set(ctre.ControlMode.PercentOutput, self.motor_output)
        self.last_motor_output = self.motor_output

        if self.clamp_on != self.last_clamp_on:
            self.clamp_arm.set(not self.clamp_on)
        if self.push_on != self.last_push_on:
            self.intake_kicker.set(self.push_on)
        if self.last_extension_on != self.extension_on:
            self.left_extension.set(self.extension_on)
            extension_double = wpilib.DoubleSolenoid.Value.kForward if self.extension_on else wpilib.DoubleSolenoid.Value.kReverse
            self.right_extension.set(extension_double)

        # Don't run the motors unless something else commands us to.
        self.motor_output = 0

        # We're not resetting the pneumatics outputs here, as once they
        # have actuated, they stay actuated without drawing power.
        self.last_clamp_on = self.clamp_on
        self.last_push_on = self.push_on
        self.last_extension_on = self.extension_on

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
        """Get the distance of the LIDAR sensor in m."""
        return self.range_finder.getDistance()

    @magicbot.feedback
    def is_cube_contained(self) -> bool:
        """Check whether a cube is in the containment mechanism."""
        if hal.isSimulation():
            return True
        cube_dist = self.get_cube_distance()
        return 0.05 <= cube_dist <= 0.35
        # return self.cube_switch.get()

    def are_wheels_contacting_cube(self) -> bool:
        """Check whether the intake wheels are touching the cube."""
        return self.left_motor.getOutputCurrent() >= 5
