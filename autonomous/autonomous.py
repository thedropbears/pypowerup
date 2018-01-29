from magicbot.state_machine import AutonomousStateMachine, state, timed_state
from pyswervedrive.swervechassis import SwerveChassis

class DriveForwardAutonomous(AutonomousStateMachine):
    MODE_NAME="CODE"
    chassis: SwerveChassis

    def __init__(self):
        super().__init__()

    @state(first=True)
    def driving_forward1(self):
        self.chassis.set_inputs(1, 0, 0)
        if self.chassis.odometry_x >= 3:
            self.next_state("stopping1")

    @timed_state(duration=2.0, must_finish=True)
    def stopping1(self):
        self.chassis.set_inputs(0, 0, 0)
        if -0.05 <= self.chassis.odometry_x_vel <= 0.05:
            self.next_state("driving_back")

    @state(must_finish=True)
    def driving_back(self):
        self.chassis.odometry_reset()
        self.chassis.set_inputs(-1, 0, 0)
        if self.chassis.odometry_x <= -1.5:
            self.next_state("driving_left")

    @state(must_finish=True)
    def driving_left(self):
        self.chassis.set_inputs(0, 1, 0)
        if self.chassis.odometry_y <= 0.5:
            self.next_state("stopping2")

    @timed_state(duration=2.0, must_finish=True)
    def stopping2(self):
        self.chassis.set_inputs(0, 0, 0)
        if -0.05 <= self.chassis.odometry_x_vel <= 0.05:
            self.next_state("driving_right")

    @state(must_finish=True)
    def driving_right(self):
        self.chassis.set_inputs(0, -1, 0)
        if self.chassis.odometry_y >=0.5:
            self.next_state("driving_forward2")

    @state(must_finish=True)
    def driving_forward2(self):
        self.chassis.set_inputs(1, 0, 0)
        if self.chassis.odometry_x >= 3:
            pass

