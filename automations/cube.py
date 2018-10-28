from components.lifter import Lifter
from components.intake import Intake
from magicbot import StateMachine, state, timed_state


class CubeManager(StateMachine):
    lifter: Lifter
    intake: Intake

    def start_match(self):
        """Initialise the intake system at the start of the match."""
        self.engage()

    @timed_state(first=True, must_finish=True, duration=0.8, next_state="drop_cube_initialize")
    def initialize_auto_cube(self):
        """Get the intake arms out of their starting position."""
        self.intake.extend_arms()
        self.intake.retract_kicker()
        self.intake.clamp()
        self.intake.outtake()

    @timed_state(must_finish=True, duration=0.5, next_state="grabbing_cube")
    def drop_cube_initialize(self):
        self.intake.unclamp()

    @timed_state(must_finish=True, duration=0.2, next_state="hovering_cube")
    def grabbing_cube(self):
        self.intake.retract_arms()
        self.intake.clamp()

    @state
    def hovering_cube(self, initial_call):
        self.lifter.move(0.1)
        self.done()

    def start_intake(self, force=False):
        """Intake a cube."""
        self.engage(initial_state='intaking_cube', force=force)

    @state(must_finish=True)
    def intaking_cube(self, initial_call):
        self.intake.unclamp()
        self.intake.retract_kicker()
        self.intake.extend_arms()
        self.intake.intake()
        if self.intake.is_cube_contained():
            self.next_state_now('pulling_in_cube')

    @timed_state(must_finish=True, duration=0.4, next_state="grabbing_cube")
    def pulling_in_cube(self, state_tm):
        self.intake.intake()
        if state_tm > 0.2:
            self.intake.retract_arms()

    def lift_to_scale(self, force=False):
        """Lift the cube to scale height."""
        self.engage(initial_state='lifting_scale', force=force)

    @state(must_finish=True)
    def lifting_scale(self, initial_call):
        if initial_call:
            self.intake.retract_arms()
            self.intake.retract_kicker()
            self.lifter.move(self.lifter.UPPER_SCALE)
        if self.lifter.at_pos():
            self.done()

    def lift_to_switch(self, force=False):
        """Lift the cube to switch height."""
        self.engage(initial_state='lifting_switch', force=force)

    @state(must_finish=True)
    def lifting_switch(self, initial_call):
        if initial_call:
            self.intake.retract_arms()
            self.intake.retract_kicker()
            self.lifter.move(self.lifter.SWITCH)
        if self.lifter.at_pos():
            self.done()

    def deposit_exchange(self, force=False):
        self.engage(initial_state='ejecting_exchange', force=force)

    @timed_state(must_finish=True, duration=1.0, next_state="reset_cube")
    def ejecting_exchange(self):
        self.intake.outtake()
        self.intake.kick()

    def eject(self, force=False):
        self.engage(initial_state='ejecting_cube', force=force)

    @timed_state(must_finish=True, duration=0.4, next_state="resetting_containment")
    def ejecting_cube(self):
        self.intake.kick()

    @state
    def resetting_containment(self):
        self.intake.unclamp()
        self.intake.retract_kicker()
        self.done()

    @timed_state(must_finish=True, duration=0.5, next_state="reset_cube")
    def waiting_to_reset(self):
        pass

    def reset(self, force=False):
        self.engage(initial_state='reset_cube', force=force)

    @state(must_finish=True)
    def reset_cube(self, initial_call):
        if initial_call:
            # self.intake.clamp(False)
            self.intake.retract_kicker()
            self.intake.retract_arms()
            self.lifter.move(self.lifter.BOTTOM_HEIGHT)
        if self.lifter.at_pos():
            self.done()

    def panic(self):
        self.engage(initial_state='panicking', force=True)

    @state(must_finish=True)
    def panicking(self):
        self.lifter.manual_down()
        self.intake.unclamp()
        self.intake.retract_kicker()
        self.intake.retract_arms()
        if self.lifter.switch_pressed():
            self.lifter.stop()
            self.done()
