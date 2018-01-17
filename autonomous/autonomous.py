"""The autonomous controls for the robot."""
from magicbot.state_machine import AutonomousStateMachine, state, timedstate
from pyswervedrive.swervechassis import SwerveChassis



class OverallBase(AutonomousStateMachine):
    """A basic statemachine that is subclassed to all required autonomous routines."""

    FMS_scale: 0  #L or R
    FMS_switch: 0  #L or R
    chassis: SwerveChassis
    def __init__(self):
        super().__init__()

    @state
    def go_to_scale(self):
        pass
        self.next_state("deposit_cube")

    @state
    def deposit_cube(self):
        pass
        self.next_state("intake_cube")

    @state
    def intake_cube(self):
        pass
        if:"""switch needs a cube, only happens once and if subclassed"""
            self.next_state("go_to_switch")
        elif: """pickup successful"""
            self.next_state("go_to_scale")


class DoubleScaleBase(OverallBase):

    def __init__(self):
        super().__init__()
        scale_first: True
        
    @state(first=True)
    def go_to_scale(self):
        pass
        self.next_state("deposit_cube")

class SwitchAndScale(OverallBase):
    
    def __init__(self):
        super().__init__()

    @state(first=True)
    def go_to_switch(self):
        """Goes to the switch, when subclassed will go to the correct side regardless of start."""
        if FMS_switch is "L"
            #go to left switch
        if FMS_switch is "R"
            #go to right switch
        pass
        self.next_state("deposit_cube")