from magicbot import StateMachine, state
from components.intake import Intake
from components.lifter import Lifter


class IntakeAutomation(StateMachine):
    intake: Intake
    lifter: Lifter

    @state(first=True, must_finish=True)
    def intake_cube(self):
        """Start the intake."""
        if self.cube_inside():
            self.intake_rotate(0.0)
            self.next_state("placing_cube")
        else:
            self.intake_rotate(0.3)

    @state(must_finish=True)
    def clamp(self):
        self.intake.intake_clamp(True)
        self.intake.intake_push(False)
        if self.intake.intake_clamp():
            self.LifterAutomation.engage()
            self.done()
        """Run lifter state machine"""
