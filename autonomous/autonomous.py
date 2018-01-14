from magicbot.state_machine import AutonomousStateMachine, state
from pyswervedrive import SwerveChassis

class DriveForwardAutonomous(AutonomousStateMachine):

    chassis: SwerveChassis

    def __init__(self):
        super().__init__()

    @state(first=True)
    def driving_forward(self):
        self.chassis.set_inputs(1, 0, 0)
        if self.chassis.odometry_x >= 5:
            self.next_state("driving_left")

    @state
    def driving_left(self):
        self.chassis.set_inputs(0, 1, 0)
        if self.odometry_y >= 5:
            self.done()

