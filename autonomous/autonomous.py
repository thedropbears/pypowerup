from magicbot.state_machine import AutonomousStateMachine, state
from pyswervedrive.swervechassis import SwerveChassis

class DriveSquareAutonoumous(AutonomousStateMachine):
    """A simple autonomuos routine designed to drive around in a square."""

    chassis: SwerveChassis
    def __init__(self):
        super().__init__()

    @state(first = True)
    def DrivingForwards(self):
        """Gives the swerve modules a positive input on the x-axis i.e. forwards."""
        self.chassis.set_inputs(1,0,0)
        if self.chassis.odometry >= 5:  #checks if the robot has moved 5 meters yet
            self.next_state('DrivingLeft')  #moves the statemachine to the next state
    @state
    def DrivingLeft(self):
        """Gives the swerve modules a positive input on the y-axis i.e. left"""
        self.chassis.set_inputs(0,1,0)
        if self.chassis.odometry >= 5:
            self.next_state('DrivingBack')
    @state
    def DrivingBack(self):
        """Gives the swerve modules a negative input on the x-axis i.e. backwards"""
        self.chassis.set_inputs(-1,0,0)
        if self.chassis.odometry >= 5:
            self.next_state('DrivingRight')
    @state
    def DrivingRight(self):
        """Gives the swerve modules a negative input on the y-axis i.e. right"""
        self.chassis.set_inputs(0,-1,0)
        if self.chassis.odometry >= 5:
            self.next_state('Stoping')
    @state
    def Stop(self):
        self.chassis.set_inputs(0,0,0)