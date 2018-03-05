from magicbot import StateMachine, state, timed_state

from components.lifter import Lifter
from components.intake import Intake


class LifterAutomation(StateMachine):
    lifter: Lifter
    intake: Intake

    @state(first=True, must_finish=True)
    def move_upper_scale(self):
        self.move_height = self.lifter.UPPER_SCALE
        self.next_state("arms_out")

    @state(must_finish=True)
    def move_balanced_scale(self):
        self.move_height = self.lifter.BALANCED_SCALE
        self.next_state("arms_out")

    @state(must_finish=True)
    def move_lower_scale(self):
        self.move_height = self.lifter.LOWER_SCALE
        self.next_state("arms_out")

    @state(must_finish=True)
    def move_switch(self):
        self.move_height = self.lifter.SWITCH
        self.next_state("arms_out")

    @timed_state(must_finish=True, duration=0.5, next_state='move')
    def arms_out(self, initial_call):
        self.intake.extend(True)
        self.intake.rotate(-1)
        self.intake.arms_out = True

    @state(must_finish=True)
    def move(self, initial_call):
        """Move to lifter height according to default height"""
        if initial_call:
            self.intake.rotate(0)
            self.lifter.move(self.move_height)
        if self.lifter.at_pos():
            self.done()

    @state(must_finish=True)
    def reset(self, initial_call):
        self.intake.extend(True)
        self.intake.arms_out = True
        self.intake.clamp(False)
        self.move_height = self.lifter.BOTTOM_HEIGHT
        self.next_state_now('move')
