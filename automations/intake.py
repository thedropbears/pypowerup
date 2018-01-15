from magicbot import StateMachine, state
from components.intake import Intake


class IntakeStateMachine(StateMachine):

    @state(first=True)
    def init(self):
        """Init the state machine.
        """
