import ms5837
import time


class DepthSensor:

    def __init__(self):
        self.reading=False
        self.sensor = ms5837.MS5837_30BA(1)

    def init(self):
        if not self.sensor.init():
            print("Sensor could not be initialized.")

    def read(self):
        if not self.sensor.read():
            print("Reading error.")
        else:
            if self.reading==False:
                self.reading=True
    def check_reading(self, reading):
        if self.reading==True:
            return reading
        else:
            print("There is no valid reading.")
            return None

    def pressure(self):
        return self.check_reading(self.sensor.pressure(ms5837.UNITS_hPa))

    def depth(self):
        return self.check_reading(self.sensor.depth())

    def temperature(self):
        return self.check_reading(self.sensor.temperature())

    def read_depth(self):
        self.read()
        return self.depth()

    def read_temperature(self):
        self.read()
        return self.temperature()

    def read_pressure(self):
        self.read()
        return self.pressure()

    def set_density(self, density):
        self.sensor.setFluidDensity(density)

if __name__=='__main__':
    ds=DepthSensor()
    ds.init()
    if ds.reading==True:
        while True:
            ds.read()
            if ds.temperature() != None:
                print("T = %.1f *C" % ds.temperature())
            if ds.pressure() != None:
                print("P = %.1f hPa" % ds.pressure())
            if ds.depth() != None:
                print("Depth =  %.3f m" % ds.depth())
            time.sleep(1)
