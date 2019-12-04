from .connectionOdroid import *
from .target import Target
import time
import threading

from .PID import *
from .connectionOdroid import *

IP_ADDRESS_1 = '10.42.0.158'  # address jetson
IP_ADDRESS_2 = '192.168.137.208'  # address odroid
PORT = 8181


lock = threading.Lock()


class Autonomy:  # to @Adam & Ernest : ej chłopaki, to narazie sama koncepcja klasy, tu będzie nasze sterowanie
    """Class that implements all autonomy -> methods that will controll AUV depending on
    what cameras see, what they saw and when and also decides in what order perform realising tasks,
    when there are many possibilities.
    Will be run in thread or maybe will inherit from Thread class"""
    def __init__(self, imu, pid_thread, conn_thread):
        self.imu = imu
        self.pid_thread = pid_thread
        self.pid_follow_obj = PID()
        # dodałem tutah obiekt connection z Jetsonem, ale raczej to zmienię i dodam jako paramert konstruktora
        # self.conn = Connection(IP_ADDRESS_2, PORT)
        self.conn = conn_thread
        self.conn.start()

        self.raw_data_frame = []
        self.objects_frame_cam1 = []
        self.objects_frame_cam2 = []
        self.objects_number_cam1 = 0
        self.objects_number_cam2 = 0
        self.objects_centras_cam1 = []
        self.objects_centras_cam2 = []
        self.objects_distances_cam1 = []
        self.objects_distances_cam2 = []



        # aktualny cel do którego zmierzamy
        self.target = Target()


    def catch_detections(self):
        # łapanie ramek danych i wpisywanie ich w pola, których bedziemy uzywac do sterowania
        while True:
            with lock:
                self.raw_data_frame = self.conn.getDataFrame()
                self.target.update_target_position(self.raw_data_frame)
                # przypisanie wartosci z ramek do tablic w klasie
                self.objects_frame_cam1 = self.raw_data_frame[0]
                self.objects_number_cam1 = len(self.objects_frame_cam1)
                self.objects_frame_cam2 = self.raw_data_frame[1]
                self.objects_number_cam2 = len(self.objects_frame_cam2)

                for object_frame in self.objects_frame_cam1:
                    self.objects_distances_cam1.append(object_frame[0])
                    self.objects_centras_cam1.append([object_frame[1], object_frame[2]])
                for object_frame in self.objects_frame_cam2:
                    self.objects_distances_cam2.append(object_frame[0])
                    self.objects_centras_cam2.append([object_frame[1], object_frame[2]])

    def look_for_detections(self):
        #self.prev_pid_values = self.pid_thread.center_x_PID.getPIDCoefficients()
        # wylacz yaw_pid, aby móc się rozejrzec
        #self.pid_thread.center_x_PID
        self.stop()
        self.pid_thread.yaw_PID.setSetPoint(0.)
        time.sleep(1)
        while not(self.target.get_flag()):
            yaw = self.imu.getSample('yaw')
            # max angle w stopniach -> zakres 'rozglądania się' łodzi
            search_max_angle_abs = 120.
            if (yaw > - search_max_angle_abs and yaw < 0) or (yaw > search_max_angle_abs):
                self.turning_left(1.)  # 1 deg/sec
                time.sleep(0.5)
            elif (yaw > 0 and yaw < search_max_angle_abs) or (yaw < - search_max_angle_abs):
                self.turning_right(1.)  # 1 deg/sec
                time.sleep(0.5)

        # tutaj jakas decyzja ktory obiekt sledzic?
        # [PRZECZYTAJ] Moze od razu szukajmy danego biektu jesli nie zdnajdziemy w danym czasie zmieniamy cel?
        # [PRZECZYTAJ] Wymaga dopracowania moze jazda do przodu po jakims czasie i tam się rozejrzenie
        # no i wywolanie sledzenia


    def turning_left(self, vel):
        self.pid_thread.yaw_PID.setSetPoint(self.pid_thread.yaw_PID.getSetPoint - vel)

    def turning_right(self, vel):
        self.pid_thread.yaw_PID.setSetPoint(self.pid_thread.yaw_PID.getSetPoint + vel)

    def follow_object(self, center_offset, pid_values, velocity):
        # for now just X pos, Y pos should be set by getting depth info
        # Nic pewnego czy zadziała narazie

        self.prev_yaw_pid_values = self.pid_thread.yaw_PID.getPIDCoefficients()
        # turn off yaw_PID, żeby mozna bylo sterowac tylko za pomocą offsetu z kamery
        self.pid_thread.yaw_PID.setPIDCoefficients(0., 0., 0.)
        self.pid_thread.center_x_PID.setPIDCoefficients(pid_values[0], pid_values[1], pid_values[2])
        while self.target.get_flag():
            obstacles = self.target.get_obstacles_to_avoid()
            if len(obstacles) == 0:
                self.pid_thread.center_x_PID.center_x_diff = self.pid_thread.center_x_PID.update(center_offset[0])  # x
                self.forward(velocity)
            else:
                pass # tu logika do wymijania obiektow

        # jeśli nie widzi obiektu przez dłuzszy czas to znowu wywołuje 'rozgladanie sie'
        # ora przywroc poprzednie nastawy PID
        # [PRZECZYTAJ] Warunek wyjścia potrzebny
        self.pid_thread.yaw_PID.setPIDCoefficients(self.prev_yaw_pid_values[0], self.prev_yaw_pid_values[1],
                                                   self.prev_yaw_pid_values[2])
        # self.look_for_detections()

    def forward(self, velocity):
        self.pid_thread.pid_motors_speeds_update[0] = velocity
        self.pid_thread.pid_motors_speeds_update[1] = velocity

    def backward(self, velocity):
        self.pid_thread.pid_motors_speeds_update[0] = -velocity
        self.pid_thread.pid_motors_speeds_update[1] = -velocity

    def stop(self):
        self.pid_thread.pid_motors_speeds_update[0] = 0
        self.pid_thread.pid_motors_speeds_update[1] = 0

    """Nawrot i beczka wymagaja wprowadzenia licznika liczby obrotow, zeby mozna bylo wyregulowac po obrocie o 360.
    Dla nawrotu nalezy w ogole przemapowac katy do domyslnych wartosci. Licznik concept: zmiana kata na imu 
    z 360 do 0 - l+=1, z -360 do 0 l-=1. W ten sposob mozna byloby zdeterminowac, że obrot zostal zrobiony czy nie.
    Na razie przed nawrotem ustawi sie do kata 0 osi Z, nastepnie obroci sie o 160 stopni, zeby nie przejsc za 180"""
    def barrel_roll(self):
        self.pid_thread.yaw_PID.setSetPoint(self.pid_thread.roll_PID.getSetPoint + 360)
        sleep(5)
        self.pid_thread.yaw_PID.setSetPoint(0)

    def nawrot(self):
        self.pid_thread.yaw_PID.setSetPoint(0)
        self.forward(800)
        sleep(3)
        self.stop()
        self.turning_right(160)
        sleep(3)
        self.forward(800)
        sleep(3)
        self.stop()

    def hit_object(self):
        pass

