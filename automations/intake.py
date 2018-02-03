from magicbot import StateMachine, state, timed_state
from components.intake import Intake
from automations.lifter import LifterAutomation


class IntakeAutomation(StateMachine):
    """Importing objects from other files"""
    intake: Intake
    lifter_automation: LifterAutomation

    @state(first=True)
    def intake_cube(self):
        """Starts the intake motors while waiting for the cube be seen by the
        infrared sensor"""
        if self.intake.cube_inside():
            self.intake.intake_rotate(0.0)
            self.intake.extension(False)
            self.done()
        else:
            self.intake.intake_rotate(0.5)
            self.intake.extension(True)

    @state()
    def clamp(self):
        """Grabs cube and starts lifter state machine"""
        self.intake.intake_clamp(True)
        self.intake.intake_push(False)
        self.lifter_automation.engage()
        self.done()

    @state()
    def waiting(self):
        if self.button_press():
            self.next_state("deposit")

    @timed_state(duration=1, next_state="reset", must_finish=True)
    def deposit(self):
        self.intake.intake_rotate(-0.5)
        self.done()
