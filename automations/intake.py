from magicbot import StateMachine, state

from components.intake import Intake


class IntakeAutomation(StateMachine):
    intake: Intake

    @state(first=True, must_finish=True)
    def starting(self):
        """Start the intake."""
        if self.cube_inside():
            self.next_state("")
            self.intake_rotate(0.0)
        else:
            self.intake_rotate(0.3)

    @state(must_finish=True)
    def placing_cube(self):
        """Placing the cube"""
        if self.cube_outside():
            self.next_state("")
            self.intake_rotate(0.0)
        else:
            self.intake_rotate(-0.3)

    @state(must_finish=True)
    def hold(self):
        """Waits for next state"""

    @state(must_finish=True)
    def stopping(self):
        """Stop the intake"""
        self.intake_disable()
