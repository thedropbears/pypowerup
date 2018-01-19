from magicbot import StateMachine, state

from components.intake import Intake


class IntakeAutomation(StateMachine):
    intake: Intake

    @state(first=True, must_finish=True)
    def starting(self):
        """Start the intake."""
        self.intake_enable(0.3)
        if self.cube_inside():
            self.next_state("stopping")

    @state(must_finish=True)
    def stopping(self):
        """Stop the intake"""
        self.intake_enable(0)
        