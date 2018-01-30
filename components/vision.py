import time
import numpy as np
from networktables import NetworkTables


class Vision:

    def setup(self):
        """This is called after variables are injected by magicbot."""
        self.nt = NetworkTables
        self.sd = self.nt.getTable("/vision")
        self.entry = self.sd.getEntry("info")
        self.entry.addListener(self.new_value, self.nt.NotifyFlags.UPDATE)

        self.vision = []

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
        start_time = time.time()
        if key == "/vision/info":
            self.unpack_info(value, start_time)

    def format_list(self, lst, n):
        out = [lst[i:i+n] for i in range(0, len(lst), n)]
        t = out[-1][0]
        out.pop(-1)
        out.append(t)
        return out

    def unpack_info(self, info, start_time):
        if len(info) <= 1:
            self.vision = [None]
            return

        info = np.array(info)
        info = self.format_list(info, 3)
        total_time = (time.time() - start_time) + float(info[-1])
        info[-1] = total_time

        self.vision = info

    def largest_cube(self):
        if len(self.vision) < 2:
            return None
        return self.vision[0][0]
