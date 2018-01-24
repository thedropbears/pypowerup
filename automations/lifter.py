from magicbot import StateMachine, state, timed_state

from components.lifter import Lifter


class LifterAutomation(StateMachine):
    lifter: Lifter

    @state(First=True, must_finish=True)
    def move(self):
        """Move to lifter height according to button press(5 buttons)"""

    @state(must_finish=True)
    def move_complete(self):
        """This state makes sure that you are at you set height"""
        if self.lifter.get_pos() == self.setpos:
            self.next_state("unload")

    @timed_state(duration=0.5, next_state="reset", must_finish=True)
    def kick(self):
        self.intake.intake_clamp(False)
        self.intake.intake_push(True)

    @state(must_finish=True)
    def reset(self):
