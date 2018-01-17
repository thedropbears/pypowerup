class Lifter:

    def setup(self):
        """This is called after variables are injected by magicbot."""
        self.switch_height = None
        self.scale_height = None
        self.exchange_height = None

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
        self.set_pos(self.switch_height)

    def move_scale(self):
        """Move the lift to the height of the scale
        """
        self.set_pos(self.scale_height)

    def move_exchange(self):
        """ Move the lift to the height of the exchange
        """
        self.set_pos(self.exchange_height)
    
    def set_pos(self, setpos):
        """Run pid loop to position on encoder

        Args:
            setpos (int): Encoder position to move lift to.
        """
        pass

    def get_pos(self):
        """Returns encoder position on lift

        Returns:
            int: The location of the lift
        """
        pass
