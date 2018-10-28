import math
from magicbot.state_machine import AutonomousStateMachine, state
from automations.motion import ChassisMotion
from pyswervedrive.chassis import Chassis


class TestOdomAuto(AutonomousStateMachine):

    MODE_NAME = "Test Odometry Auto"

    chassis: Chassis
    motion: ChassisMotion

    @state(first=True)
    def move_forward(self, initial_call):
        if initial_call:
            self.chassis.odometry_x = 0.44
            self.motion.set_trajectory([[self.chassis.odometry_x, self.chassis.odometry_y],
                                        [3, 0]], end_heading=0)
        if not self.motion.trajectory_executing:
            self.next_state_now('move_back')

    @state
    def move_back(self, initial_call):
        if initial_call:
            self.motion.set_trajectory([[self.chassis.odometry_x, self.chassis.odometry_y],
                                        [0.44, 0]], end_heading=math.pi/2)
        if not self.motion.trajectory_executing:
            self.next_state_now('move_forward')
