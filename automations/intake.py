from magicbot import StateMachine, state
from components.intake import Intake
from components.lifter import Lifter
from automations.lifter import LifterAutomation


class IntakeAutomation(StateMachine):
    intake: Intake
    lifter: Lifter
    lifter_automation: LifterAutomation

    @state(First=True, must_finish=True)
    def intake_cube(self):
        """Start the intake while waiting for the cube to come inside"""
        if self.cube_inside():
            self.intake_rotate(0.0)
            self.next_state("clamp")
        else:
            self.intake_rotate(0.3)

    @state(must_finish=True)
    def clamp(self):
        """Grabs cube and Run lifter state machine"""
        self.intake.intake_clamp(True)
        self.intake.intake_push(False)
        self.lifter_automation.engage()
        self.done()
