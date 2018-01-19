from magicbot import StateMachine, state

from components.intake import Intake


class IntakeAutomation(StateMachine):
    intake: Intake

    @state(first=True, must_finish=True)
    def intake_cube(self):
        """Start the intake."""
        if self.cube_inside():
            self.intake_rotate(0.0)
            self.next_state("placing_cube")
        else:
            self.intake_rotate(0.3)

    @state(must_finish=True)
    def hold(self):
        """Waits for next state, lifter to reach position
        then runs placing_cube state"""

    @state(must_finish=True)
    def place_cube(self):
        """Placing the cube"""
        if self.cube_outside():
            self.intake_rotate(0.0)
            self.next_state("intake_cube")
        else:
            self.intake_rotate(-0.3)

    @state(must_finish=True)
    def stopping(self):
        """Stop the intake"""
        self.intake_disable()
