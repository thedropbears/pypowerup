from magicbot import StateMachine, state, timed_state

from components.lifter import Lifter
from components.intake import Intake

 
class LifterAutomation(StateMachine):
    lifter: Lifter
    intake: Intake

    @state(first=True)
    def move(self):
        """Move to lifter height according to button press(5 buttons)"""
        self.lifter.move()
        self.next_state("move_complete")

    @state()
    def move_complete(self):
        """This state makes sure that you are at you set height."""
        if self.lifter.at_pos():
            self.next_state("eject")

    @timed_state(duration=0.5, next_state="reset", must_finish=True)
    def eject(self):
        """Ejects cube from mechanism as height is reached."""
        self.intake.intake_clamp(False)
        self.intake.intake_push(True)

    @state(must_finish=True)
    def reset(self):
        self.intake.intake_push(False)
        self.lifter.reset_pos()
