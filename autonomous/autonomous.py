"""The autonomous controls for the robot. Vision code is not yet included"""
import wpilib
from magicbot.state_machine import AutonomousStateMachine, state
from pyswervedrive.swervechassis import SwerveChassis



class OverallBase(AutonomousStateMachine):
    """A basic statemachine that is subclassed to all required autonomous routines."""

    FMS_scale = 0  #L or R
    FMS_switch = 0  #L or R
    chassis = SwerveChassis

    @state(first=True)
    def go_to_scale(self):
        """The robot travels to the scale"""
        if self.FMS_scale == 'L':
            #go to left scale
            pass
        if self.FMS_scale == 'R':
            #go to right scale
            pass
        self.next_state("deposit_cube")

    @state
    def deposit_cube(self):
        """The robot releases its cube into either the scale or switch"""
        self.next_state("find_cube")

    @state
    def find_cube(self):
        """The robot drives towards where the next cube should be"""
        if 1 == 1:  #Gets to cube position
            self.next_state("search_for_cube")

    @state
    def search_for_cube(self):
        """The robot attemppts to find a cube within the frame of the camera"""
        if 1 == 1:  #cube found
            self.next_state("cube_in_frame")

    @state
    def cube_in_frame(self):
        """The robot has located a cube within view and rotates towards it"""
        if 1 == 1:  #On target
            self.next_state("drive_forwards")
        if 2 == 2:  #Off target
            self.next_state("rotate_to_cube")

    @state
    def drive_forwards(self):
        """Drives forward and attempts to intake the cube"""
        #Run intake
        if 1 == 1:  #cube inside intake system (current spikes)
            self.next_state("go_to_scale")
        elif 2 == 2:  #After completing the intake cycle there is no cube
            self.next_state("search_for_cube")

    @state
    def rotate_to_cube(self):
        """The robot rotates in the direction specified by the vision system"""
        if  1 == 1:  #Cube angle is negative
            #rotate counterclockwise (confirm this)
            pass
        if  2 == 2:  #Cube angle is positive
            #rotate clockwise (confirm this)
            pass
        self.next_state("search_for_cube")

#class DoubleScale(OverallBase):
#    """A less general routine for the double-scale strategy. Still requires subclassing"""


class SwitchAndScale(OverallBase):
    """A less general routine for the switch and scale strategy. Still requires subclassing"""
    been_to_switch = False

    @state
    def go_to_switch(self):
        """Goes to the switch, when subclassed will go to the correct side regardless of start."""
        self.been_to_switch = True
        if self.FMS_switch == 'L':
            #go to left switch
            pass
        if self.FMS_switch == 'R':
            #go to right switch
            pass
        self.next_state("deposit_cube")

    @state
    def intake_cube(self):
        """The robot drives towards where the next cube should be and then hands
        over to vision for fine tuning"""
        if self.been_to_switch is False:  #switch needs a cube and pickup was successful
            self.next_state("go_to_switch")
        elif self.been_to_switch:  #The switch has a cube and pickup was successful
            self.next_state("go_to_scale")

class LeftSwitchAndScale(SwitchAndScale):
    """The switch and scale strategy when we start on the left"""
    MODE_NAME = 'Switch and scale - left start'
    start_position = 0  #change to left start position

    @state(first=True)
    def go_to_scale(self):
        if self.FMS_switch == 'L':
            self.next_state("go_to_switch")
        elif self.FMS_scale == 'R':
            #go to right scale
            pass
        elif self.FMS_scale == 'L':
            #go to left scale
            pass
        self.next_state("deposit_cube")

class RightSwitchAndScale(SwitchAndScale):
    """The switch and scale strategy when we start on the right"""
    MODE_NAME = 'Switch and scale - right start'
    start_position = 0  #change to right start position

    @state(first=True)
    def go_to_scale(self):
        if self.FMS_switch == 'R':
            self.next_state("go_to_switch")
        elif self.FMS_scale == 'R':
            #go to right scale
            pass
        elif self.FMS_scale == 'L':
            #go to left scale
            pass
        self.next_state("deposit_cube")

class LeftDoubleScale(OverallBase):
    """The double scale strategy when we start on the left"""
    MODE_NAME = 'Double scale - left start'
    start_position = 0 #change to left start position
    #I think i need more here but am not sure

class RightDoubleScale(OverallBase):
    """The double scale strategy when we start on the right"""
    MODE_NAME = 'Double scale - right start'
    start_position = 0 #change to right start position
    #I think i need more here but am not sure
