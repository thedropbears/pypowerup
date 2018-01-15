from magicbot import StateMachine, state
from components.lifter import Lifter


class LifterStateMachine(StateMachine):

    @state(first=True)
    def init(self):
        """Init the state machine.
        """
