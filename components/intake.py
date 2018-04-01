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
        for motor in self.left_motor, self.right_motor:
            motor.setNeutralMode(ctre.NeutralMode.Coast)
            motor.configPeakCurrentLimit(50, timeoutMs=10)
            motor.configContinuousCurrentLimit(50, timeoutMs=10)
            motor.configPeakCurrentDuration(500, timeoutMs=10)
            motor.enableCurrentLimit(True)

        self.right_motor.follow(self.left_motor)
        self.right_motor.setInverted(True)

    def on_enable(self):
        """This is called whenever the robot transitions to being enabled."""
        self.last_clamp_on = None
        self.last_push_on = None
        self.last_extension_on = None
        self.last_motor_output = None

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

    def intake(self):
        """Spin the intake wheels inwards."""
        self.motor_output = -1

    def outtake(self):
        """Spin the intake wheels outwards."""
        self.motor_output = 1

    def clamp(self):
        """Clamp the cube."""
        self.clamp_on = True

    def unclamp(self):
        """Unclamp the cube."""
        self.clamp_on = False

    def toggle_clamp(self):
        """Toggle whether the cube should be clamped.

        This really shouldn't be necessary...
        """
        self.clamp_on = not self.clamp_on

    def push(self, value: bool):
        """Turn the pushing pneumatic on or off."""
        self.push_on = value

    def kick(self):
        """
        Kick the cube out of the containment.

        This will automatically unclamp the cube.
        """
        self.clamp_on = False
        self.push_on = True

    def retract_kicker(self):
        """Retract the cube kicker."""
        self.push_on = False

    def extend_arms(self):
        """Extend the intake arms."""
        self.extension_on = True

    def retract_arms(self):
        """Retract the intake arms."""
        self.extension_on = False

    def toggle_arms(self):
        """Toggle whether the arms are extended."""
        self.extension_on = not self.extension_on

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
