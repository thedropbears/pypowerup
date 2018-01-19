from magicbot import StateMachine, state

from components.intake import Intake


class IntakeAutomation(StateMachine):
    intake: Intake

    @state(first=True)
    def starting(self):
        """Start the intake."""
        ...
