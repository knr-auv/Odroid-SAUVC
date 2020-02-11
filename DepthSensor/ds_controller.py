import time

from depth_sensor import DepthSensor


class DSController:

    def __init__(self):
        self.ds = DepthSensor()
        self.ds.init()
        self.temp = 0
        self.pressure = 0
        self.depth = 0

    def set_ref(self):
        self.ds.set_ref_p()

    def read_temperature(self):
        self.temp = self.ds.read_temperature()
        return self.temp

    def read_pressure(self, it=3):
        p = 0
        for i in range(it):
            p += self.ds.read_pressure()
        self.pressure = p / it
        return self.pressure

    def read_depth(self, it=3):
        d = 0
        for i in range(it):
            d += self.ds.read_depth()
        self.depth = d / it
        return self.depth


if __name__ == '__main__':
    ds = DSController()
    while True:
        print("Depth %.2f m" % ds.read_depth())
        time.sleep(0.2)
