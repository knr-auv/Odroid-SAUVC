import random
import threading
from server import *


IP_ADDRESS_2 = '10.42.0.158'  # address odroid

PLOT_PORT = 8200

plot_data = [0, 0, 0, 0, 0, 0]


class IMUThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()

    def run(self):
        # will start printing samples (maybe we could run it in another terminal)
        while True:
            with self.lock:
                plot_data[0] = random.randint(-180, 180)
                plot_data[1] = random.randint(-180, 180)
                plot_data[2] = random.randint(-180, 180)


class PIDThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()

    def run(self):
        # will start printing samples (maybe we could run it in another terminal)
        while True:
            with self.lock:
                plot_data[3] = 2
                plot_data[4] = 2
                plot_data[5] = 2


class PlotThread(threading.Thread):
    def __init__(self, ip, port):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()

        # stworzenie servera
        self.server = Server(ip, port)



    def run(self):
        global plot_data
        while True:
            with self.lock:
                self.server.sendData(plot_data)
                print(plot_data)


imu = IMUThread()
pid = PIDThread()
plot = PlotThread(IP_ADDRESS_2, PLOT_PORT)

plot.start()
imu.start()
pid.start()
