from components.lifter import Lifter
from components.intake import Intake
from magicbot import StateMachine, state, timed_state


class CubeManager(StateMachine):
    lifter: Lifter
    intake: Intake

    @timed_state(first=True, must_finish=True, duration=0.5, next_state="grabbing_cube")
    def initialize_auto_cube(self):
        """Get the intake arms out of their starting position."""
        self.intake.extend(True)
        self.intake.push(False)
        self.intake.clamp(False)
        self.intake.rotate(1)
        self.intake.arms_out = True

    @timed_state(must_finish=True, duration=0.2, next_state="hovering_cube")
    def grabbing_cube(self):
        self.intake.rotate(0)
        self.intake.extend(False)
        self.intake.clamp(True)

    @state(must_finish=True)
    def hovering_cube(self, initial_call):
        self.lifter.move(0.1)
        self.done()

    @state(must_finish=True)
    def intaking_cube(self, initial_call):
        self.intake.clamp(False)
        self.intake.push(False)
        self.intake.extend(True)
        self.intake.rotate(-1)
        if self.intake.is_cube_contained():
            self.next_state_now('pulling_in_cube')

    @timed_state(must_finish=True, duration=0.4, next_state="grabbing_cube")
    def pulling_in_cube(self, state_tm):
        self.intake.rotate(-1)
        if state_tm > 0.2:
            self.intake.extend(False)

    @state(must_finish=True)
    def lifting_scale(self, initial_call):
        if initial_call:
            self.intake.extend(False)
            self.intake.push(False)
            self.lifter.move(self.lifter.UPPER_SCALE)
        if self.lifter.at_pos():
            self.done()

    @state(must_finish=True)
    def lifting_switch(self, initial_call):
        if initial_call:
            self.intake.extend(False)
            self.intake.push(False)
            self.lifter.move(self.lifter.SWITCH)
        if self.lifter.at_pos():
            self.done()

    @timed_state(must_finish=True, duration=1.0, next_state="reset_cube")
    def ejecting_exchange(self):
        self.intake.rotate(1)
        self.intake.clamp(False)
        self.intake.push(True)

    @timed_state(must_finish=True, duration=0.4, next_state="resetting_containment")
    def ejecting_cube(self):
        self.intake.clamp(False)
        self.intake.push(True)

    @state
    def resetting_containment(self):
        self.intake.clamp(False)
        self.intake.push(False)
        self.done()

    @timed_state(must_finish=True, duration=0.5, next_state="reset_cube")
    def waiting_to_reset(self):
        pass

    @state(must_finish=True)
    def reset_cube(self, initial_call):
        if initial_call:
            self.intake.clamp(False)
            self.intake.push(False)
            self.intake.extend(False)
            self.lifter.move(self.lifter.BOTTOM_HEIGHT)
        if self.lifter.at_pos():
            self.done()
