from magicbot import StateMachine, state

from components.lifter import Lifter


class LifterAutomation(StateMachine):
    lifter: Lifter

    @state(first=True)
    def starting(self):
        """Start the lifter."""
        ...
