import time
import hal

from networktables import NetworkTables


class Vision:
    CAMERA_HEIGHT = 0.42  # m

    def __init__(self):
        """This is called after variables are injected by magicbot."""
        self.nt = NetworkTables.getTable("/vision")
        self.entry = self.nt.getEntry("info")
        self.entry.addListener(self.new_value, NetworkTables.NotifyFlags.NEW | NetworkTables.NotifyFlags.UPDATE)

        self.time = 0
        self.data = []

    def on_enable(self):
        """This is called whenever the robot transitions to being enabled."""
        pass

    def on_disable(self):
        """This is called whenever the robot transitions to disabled mode."""
        pass

    def execute(self):
        """Run at the end of every control loop iteration."""
        pass

    def new_value(self, entry, key, value, param):
        self.time = time.monotonic()
        self.data = value

    def largest_cube(self):
        if hal.isSimulation():
            return 0
        if len(self.data) < 2:
            return None
        return self.data[0]
