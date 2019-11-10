from .connectionOdroid import *
from .target import Target
import time

IP_ADDRESS_1 = '10.42.0.158'  # address jetson
IP_ADDRESS_2 = '192.168.137.208'  # address odroid
PORT = 8181


class Autonomy:  # to @Adam & Ernest : ej chłopaki, to narazie sama koncepcja klasy, tu będzie nasze sterowanie
    """Class that implements all autonomy -> methods that will controll AUV depending on
    what cameras see, what they saw and when and also decides in what order perform realising tasks,
    when there are many possibilities.
    Will be run in thread or maybe will inherit from Thread class"""
    def __init__(self, imu_thread, pid_thread):
        self.imu_thread = imu_thread
        self.pid_thread = pid_thread

        # dodałem tutah obiekt connection z Jetsonem, ale raczej to zmienię i dodam jako paramert konstruktora
        self.conn = Connection(IP_ADDRESS_2, PORT)
        self.conn.start()

        self.objects_detected_frame = []
        self.objects_number = 0
        self.objects_centras = []
        # and more like that...

        # aktualny cel do którego zmierzamy
        self.target = Target()


    def catch_detections(self):
        while True:
            self.objects_detected_frame = self.conn.getDataFrame()
            # przypisanie wartosci z ramek do tablic w klasie
            self.target.update_target_position(self.objects_detected_frame)  # aktualizacja targetu

    def follow_object(self):
        pass

    def forward(self):
        pass

    def backward(self):
        pass

    def barrel_roll(self):
        pass

    def hit_object(self):
        pass

