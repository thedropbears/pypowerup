from magicbot import StateMachine, state, timed_state
from components.intake import Intake


class TestAutomation(StateMachine):
    intake: Intake

    @timed_state(first=True, duration=5, next_state="kick", must_finish=True)
    def clamp(self):
        self.intake.intake_clamp(True)

    @timed_state(duration=1, next_state="reset", must_finish=True)
    def kick(self):
        self.intake.intake_clamp(False)
        self.intake.intake_push(True)

    @state(must_finish=True)
    def reset(self):
        self.intake.intake_push(False)
        self.done()
