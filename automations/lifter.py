from magicbot import StateMachine, state

from components.lifter import Lifter

from components.intake import Intake

from automations.intake import Intake


class LifterAutomation(StateMachine):
    lifter: Lifter

    @state(must_finish=True, Finish=True)
    def cube_detection(self):
        """Check for cube inside mechanism."""
        if cube_inside():
            self.next_state("move")

    @state(must_finish=True)
    def move(self):
        """Move to Scale height."""
        if scaleButton:
            self.setpos = self.lifter.scale_height
            self.lifter.move_scale()
        elif switchButton:
            self.setpos = self.lifter.switch_height
            self.lifter.move_switch()

    @state(must_finish=True)
    def move_complete(self):
        """Move to Switch height."""
        if self.lifter.get_pos() == self.setpos:
            self.next_state("unload")

    @state(must_finish=True)
    def unload(self):
        if unloadButton:
            place_cube()
            self.done