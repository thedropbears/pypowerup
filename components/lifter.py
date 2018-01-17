class Lifter:

        def __init__(self):
            super().__init__(self)

        def setup(self):
            """Run just after createObjects.
            Useful if you want to run something after just once after the
            robot code is started, that depends on injected variables.
            """
            pass

        def on_enable(self):
            """Run every time the robot transitions to being enabled.
            """
            pass

        def on_disable(self):
            """Run every time the robot transitions to being disabled.
            """
            pass

        def execute(self):
            """Run at the end of every control loop iteration.
            """
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
