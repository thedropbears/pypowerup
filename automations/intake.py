from magicbot import StateMachine, state
from components.intake import Intake
from automations.lifter import LifterAutomation


class IntakeAutomation(StateMachine):
    """Importing objects from other files"""
    intake: Intake
    lifter_automation: LifterAutomation

    @state(first=True)
    def intake_cube(self):
        """Starts the intake motors while waiting for the cube be seen by the infrared sensor"""
        if self.cube_inside():
            self.intake_rotate(0.0)
            self.intake.extension(False)
            self.next_state("clamp")
        else:
            self.intake_rotate(0.5)
            self.intake.extension(True)

    @state()
    def clamp(self):
        """Grabs cube and starts lifter state machine"""
        self.intake.intake_clamp(True)
        self.intake.intake_push(False)
        self.lifter_automation.engage()
        self.done()
