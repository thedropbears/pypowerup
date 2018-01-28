from magicbot import StateMachine, state
from components.intake import Intake
from automations.lifter import LifterAutomation


class IntakeAutomation(StateMachine):
    intake: Intake
    lifter_automation: LifterAutomation

    @state(first=True, must_finish=True)
    def intake_cube(self):
        """Start the intake while waiting for the cube to come inside"""
        if self.cube_inside():
            self.intake_rotate(0.0)
            self.intake.extension(False)
            self.next_state("clamp")
        else:
            self.intake_rotate(0.5)
            self.intake.extension(True)

    @state(must_finish=True)
    def clamp(self):
        """Grabs cube and starts lifter state machine"""
        self.intake.intake_clamp(True)
        self.intake.intake_push(False)
        self.lifter_automation.engage()
        self.done()
