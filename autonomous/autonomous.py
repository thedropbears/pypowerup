"""The autonomous controls for the robot."""
import time
from magicbot.state_machine import AutonomousStateMachine, state, timedstate
from pyswervedrive.swervechassis import SwerveChassis



class DriveSquareAutonoumous(AutonomousStateMachine):
    """A simple autonomuos routine designed to drive around in a square."""

    chassis = SwerveChassis

    def __init__(self):
        super().__init__()

    @state(first=True)
    def driving_forwards(self):
        """Gives the swerve modules a positive input on the x-axis i.e.
        forwards."""
        self.chassis.set_inputs(1, 0, 0)
        if self.chassis.odometry >= 5:
            # checks if the robot has moved 5 meters yet
            self.next_state('driving_left')
            # moves the statemachine to the next state

    @state
    def driving_left(self):
        """Gives the swerve modules a positive input on the y-axis i.e.
        left"""
        self.chassis.set_inputs(0, 1, 0)
        if self.chassis.odometry >= 5:
            self.next_state('driving_back')

    @state
    def driving_back(self):
        """Gives the swerve modules a negative input on the x-axis i.e.
        backwards"""
        self.chassis.set_inputs(-1, 0, 0)
        if self.chassis.odometry >= 5:
            self.next_state('driving_right')

    @state
    def driving_right(self):
        """Gives the swerve modules a negative input on the y-axis i.e.
        right"""
        self.chassis.set_inputs(0, -1, 0)
        if self.chassis.odometry >= 5:
            self.next_state('spin_counter_clock')

    @timedstate
    def spin_counter_clock(self, duration=10):
        """makes the robot spin WIP"""
        while duration > 0:
            self.chassis.set_inputs(0, 0, 1)
            time.sleep(1)
        self.next_state('stop')

    @state
    def stop(self):
        """Stops the robot from moving at the end of the routine"""
        self.chassis.set_inputs(0, 0, 0)
