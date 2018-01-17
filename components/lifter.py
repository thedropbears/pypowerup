class Lifter:

    def setup(self):
        """This is called after variables are injected by magicbot."""
        pass

    def on_enable(self):
        """This is called whenever the robot transitions to being enabled."""
        pass

    def on_disable(self):
        """This is called whenever the robot transitions to disabled mode."""
        pass

    def execute(self):
        """Run at the end of every control loop iteration."""
        pass

    def move_switch(self):
        """Move the lift to the height of the switch
        """
        pass

    def move_scale(self):
        """Move the lift to the height of the scale
        """
        pass

    def move_exchange(self):
        """ Move the lift to the height of the exchange
        """
        pass
    
    def set_pos(self, setpos):
        """Run pid loop to position on encoder
        """
        pass

    def get_pos(self):
        """Returns encoder position
        """
        pass