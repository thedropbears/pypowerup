from magicbot import StateMachine, state, timed_state
from components.intake import Intake


class TestAutomation(StateMachine):
    intake: Intake

    @timed_state(first=True, duration=2, must_finish=True)
    def closing(self):
        self.arm(True)
        self.next_state("shoot")

    @timed_state(duration=2, must_finish=True)
    def shoot(self):
        self.arm(False)
        self.shoot(True)
        self.next_state("disable")

    @state(must_finish=True)
    def disable(self):
        self.shoot(False)
