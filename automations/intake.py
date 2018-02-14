from magicbot import StateMachine, state, timed_state
from components.intake import Intake
from automations.lifter import LifterAutomation


class IntakeAutomation(StateMachine):
    """Importing objects from other files"""
    intake: Intake
    lifter_automation: LifterAutomation

    def on_enable(self):
        if not self.intake.arms_out:
            self.engage(initial_state="start")

    @timed_state(must_finish=True, duration=1, next_state="stop")
    def start(self, state_tm):
        if state_tm < 0.5:
            self.intake.extension(True)
            self.intake.rotate(1)
            self.intake.arms_out = True
        else:
            self.intake.extension(False)
            self.intake.intake_clamp(False)
            self.intake.intake_push(False)

    @state(first=True, must_finish=True)
    def intake_cube(self, state_tm):
        """Starts the intake motors while waiting for the cube be seen by the
        infrared sensor"""
        if self.intake.seeing_cube() and state_tm > 0.5:
            self.intake.intake_clamp(False)
            self.next_state("pulling_in_cube")
        else:
            self.intake.rotate(-1)
            self.intake.extension(True)

    @timed_state(must_finish=True, duration=1, next_state="grab_cube")
    def pulling_in_cube(self):
        self.intake.extension(False)
        self.intake.rotate(-1)

    @state(must_finish=True)
    def grab_cube(self):
        self.intake.intake_clamp(True)
        self.intake.intake_push(False)
        self.next_state("stop")

    @state(must_finish=True)
    def deposit(self):
        """Deposit cube."""
        if self.intake.seeing_cube():
            self.next_state("push_out_cube")
        else:
            self.intake.rotate(1)
            self.intake.intake_push(True)
            self.intake.intake_clamp(False)

    @timed_state(must_finish=True, duration=1, next_state="stop")
    def push_out_cube(self):
        self.intake.rotate(1)
        self.intake.extension(False)
        self.intake.intake_push(False)

    @timed_state(must_finish=True, duration=0.4, next_state="reset_containment")
    def eject_cube(self):
        self.intake.intake_clamp(False)
        self.intake.intake_push(True)

    @state(must_finish=True)
    def reset_containment(self):
        self.intake.intake_clamp(False)
        self.intake.intake_push(False)
        self.done()

    @state(must_finish=True)
    def stop(self):
        """Stop moving motor."""
        self.intake.rotate(0)
        self.intake.extension(False)
        self.done()
