from magicbot.state_machine import AutonomousStateMachine, state
from pyswervedrive.swervechassis import SwerveChassis


class CrossBaseline(AutonomousStateMachine):

    chassis: SwerveChassis
    MODE_NAME = 'Drive forwards'
    DEFAULT = True

    @state(first=True)
    def go_forward(self, initial_call):
        if initial_call:
            self.chassis.set_inputs(3, 0, 0)
        if self.chassis.odometry_x >= 3:
            self.next_state('stopping')

    @state
    def stopping(self):
        self.chassis.set_inputs(0, 0, 0)
