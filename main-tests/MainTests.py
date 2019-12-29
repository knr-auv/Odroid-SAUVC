import time
import threading
import pickle
import os
from imu import IMUClass
from  MotorControl import *
import DHT
#from connectionForTesting import *
from Integrator import *
from connectionOdroid import *
from server import Server

# it would be wise to use logging afterwards with more complicated code
# import logging

# logging.basicConfig(filename='output.log', level=logging.INFO)

PAD_STEERING_FLAG = False

SAVE_FLAG = False
READ_FLAG = True
MOVES_FILE = "moves.dat"


IP_ADDRESS_2 = '10.42.0.158'  # address jetson
IP_ADDRESS_1 = '192.168.137.208'  # address odroid

PAD_PORT = 8186

RUN_FORWARD_VALUE = 400.

from PID import PID

# horizontal left / right, vertical left/mid/right
motors_names = ['hl', 'hr', 'vl', 'vm', 'vr']   # should be connected in this order to pwm outputs 0, 1, 2,...

# list where motors' speeds will be stored
motors_speed = [0, 0, 0, 0, 0]

# list of changes to motors' speeds made by PIDs
motors_speed_diff_pid = [0, 0, 0, 0, 0]

# list of changes to motors' speeds made by pad
motors_speed_pad = [0, 0, 0, 0, 0]

# roll, pitch, yaw angles
RPY_angles = [0, 0, 0]

run_flag = False


class DHTThread(threading.Thread):
    """Thread class for triggering and handling DHT sensor input"""
    def __init__(self):
        threading.Thread.__init__(self)
        self.temperature = 0
        self.humidity = 0
        # self.instance = DHT.DHT11(6)

    def run(self):
        while True:
            pass
            # result = self.instance.read()
            # if result.is_valid():
            #     self.humidity = result.humidity
            #     self.temperature = result.temperature
            #     print("Temp: {}\tHumid: {}".format(self.temperature, self.humidity))
            time.sleep(2)


class MotorsControlThread(threading.Thread):
    """Thread class for setting and updating thrusters' velocity
    velocity is being given as value from -1000 to 1000 (pulse width)
    in comments name 'motors' and 'thrusters' are equal"""
    def __init__(self):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.motors = MotorControl(526)    # check freqency, maybe put slightly different one
        self.motors.initialize_all()   # thrusters need initialization process before running
        global motors_speed_diff_pid
        motors_speed = [0, 0, 0, 0, 0]
        global run_flag
    def run(self):
        while True:
            with self.lock:
                # prints velocity given to motors (thrusters)
                #print('Main:')
                #print(motors_speed_diff_pid)
                #print('---')
                #motors_speed = [0, 0, 0, 0, 0]
                #print("NA SILNIKI:")
                #print(motors_speed)    # comment
                for i in range(5):
                    motors_speed[i] += motors_speed_diff_pid[i]
                    #if (i ==0  or i ==1) and run_flag:
                    #    motors_speed[i] += RUN_FORWARD_VALUE  # previous version of "run" command execution - test new and del this
                    motors_speed_diff_pid[i] = 0
                    self.motors.run_motor(i, motors_speed[i])
                    #print("silnik {}, wypelnienie {}".format(i, motors_speed[i]))
                    motors_speed[i] = 0    # uncomment
                    #print("{}:{}".format(motors_names[i], motors_speed[i]), end=" ")    # comment
                #print(motors_speed)
                time.sleep(0.2)    # comment


class IMUThread(threading.Thread):
    """Thread class that reads IMU's input and prints it for testing reasons"""
    def __init__(self, imu):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.IMU = imu

    def run(self):
        # will start printing samples (maybe we could run it in another terminal)
        c = 0
        while True:
            
            self.IMU.catchSamples()
            #self.connObj.setDataFrame(self.IMU.getSamples())
            self.IMU.printSamples(c % 50 == 0)
            c += 1

    def getIMU(self):
        return self.imu

    def setIMU(self, imu):
        self.IMU = imu

    # Method for setting connection with PC to visualize 3D position of AUV
    # Not used yet because of PC script's issues
    def setConnection(self, connObject):
        self.connObj = connObject

class PIDThread(threading.Thread):
    """Thread class that sets up all PID controllers and updates motors velocity after calculating difference
    between actual position and set_point position
    roll
    pitch
    yaw
    depth - not used yet because of lack of depth feedback in AUV
    velocity - not used because of lack of velocity feedback in AUV"""
    def __init__(self):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.roll_PID = PID()
        self.pitch_PID = PID()
        self.yaw_PID = PID()
        self.velocity_PID = PID()
        #TODO: depth_PID object and things related to it

        #Camera PID
        self.center_y_PID = PID()
        self.center_y_diff = 0

        self.center_x_PID = PID()
        self.center_x_diff = 0

        self.integrator = Integrator()
        global motors_speed_diff_pid, motors_speed_pad
        self.IMU = None
        self.roll_diff, self.pitch_diff, self.yaw_diff, self.velocity_diff = 0, 0, 0, 0

        max_sum_output = 900.
        self.roll_PID.setMaxOutput(max_sum_output / 4)
        self.pitch_PID.setMaxOutput(max_sum_output / 4)
        self.yaw_PID.setMaxOutput(max_sum_output / 4)
        self.velocity_PID.setMaxOutput(max_sum_output / 4)

        self.pid_motors_speeds_update = [0, 0, 0, 0, 0]

    def run(self):
        while True:
            with self.lock:
                self.roll_diff = self.roll_PID.update(self.IMU.getSample('roll'))
                self.pitch_diff = self.pitch_PID.update(self.IMU.getSample('pitch'))
                self.yaw_diff = self.yaw_PID.update(self.IMU.getSample('yaw'))  # maybe try:  'gyro_raw_x' 'gro_proc_x'

                #self.velocity_diff = self.velocity_PID.update(self.IMU.getSample('vel_x'))

                # prints for testing reasons
                #print(self.roll_diff)
                #print(self.pitch_diff)
                #print(self.yaw_diff)
                self.roll_control()
                self.pitch_control()
                self.yaw_control()
                if PAD_STEERING_FLAG or READ_FLAG:
                    self.pad_control()
                self.center_x_control()
                self.center_y_control()
                #self.velocity_control()
                self.update_motors()
                time.sleep(0.2)


    def roll_control(self):
        self.pid_motors_speeds_update [4] -= self.roll_diff
        self.pid_motors_speeds_update [2] += self.roll_diff

    def pitch_control(self):
        self.pid_motors_speeds_update[2] += self.pitch_diff  # * 2 / 3
        self.pid_motors_speeds_update[4] += self.pitch_diff  # * 2 / 3
        self.pid_motors_speeds_update[3] -= self.pitch_diff

    def yaw_control(self):
        global run_flag
        self.pid_motors_speeds_update[0] += self.yaw_diff
        self.pid_motors_speeds_update[1] -= self.yaw_diff
        if run_flag:
            self.pid_motors_speeds_update[0] += RUN_FORWARD_VALUE
            self.pid_motors_speeds_update[1] += RUN_FORWARD_VALUE

    def pad_control(self):
        self.pid_motors_speeds_update[0] += motors_speed_pad[0]
        self.pid_motors_speeds_update[1] += motors_speed_pad[1]
        self.pid_motors_speeds_update[2] += motors_speed_pad[2]
        self.pid_motors_speeds_update[3] += motors_speed_pad[3]
        self.pid_motors_speeds_update[4] += motors_speed_pad[4]

    # not used yet
    def velocity_control(self):
        self.pid_motors_speeds_update[0] -= self.velocity_diff  # minusy bo silniki zamontowane odwrotnie?
        self.pid_motors_speeds_update[1] -= self.velocity_diff

    def center_x_control(self):
        self.pid_motors_speeds_update[0] -= self.center_x_diff
        self.pid_motors_speeds_update[1] += self.center_x_diff

    def center_y_control(self):
        self.pid_motors_speeds_update[2] += self.center_y_diff
        self.pid_motors_speeds_update[3] += self.center_y_diff
        self.pid_motors_speeds_update[4] += self.center_y_diff

    # method that updates motors velocity
    # you can pass velocity to pid_motors_speeds_update in cose to set the velocity on motors without PID controller
    # turned on
    def update_motors(self):
        #print(self.pid_motors_speeds_update)
        motors_speed_diff_pid[:] = self.pid_motors_speeds_update[:]
        #print('Po przypisaniu:')
        #print(motors_speed_diff_pid)
        self.pid_motors_speeds_update = [0] * 5

    def getIMU(self):
        return self.IMU

    def setIMU(self, imu):
        self.IMU = imu

class MotorsWaitThread(threading.Thread):
    """Thread class only for testing
    Should be deleted soon"""
    wait_time = 0
    prev_speed = []

    def __init__(self, wait_time, prev_speed):
        threading.Thread.__init__(self)
        self.wait_time = wait_time
        self.prev_speed = prev_speed

    def run(self):
        print("Running motors for {}s".format(self.wait_time))
        time.sleep(self.wait_time)
        print("Back to: {}".format(self.prev_speed))
        motors_speed[:] = self.prev_speed[:]


class UIThread(threading.Thread):
    """Thread class that is terminal's UI for setting PIDs, primitive steering by commands """
    def __init__(self, pid_thread):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.prev_speed = motors_speed[:]
        self.motors_wait_thread = MotorsWaitThread(0, [])
        self.pid_thread = pid_thread

    def run(self):
        global motors_speed, motors_names, run_flag

        while True:
            cmd = input()
            args = cmd.split(' ')
            print(args)

            with self.lock:
                if len(args) == 3:
                    self.prev_speed[:] = motors_speed[:]

                if args[0] == "s":
                    print("Stopping all motors")
                    motors_speed = [0] * 5
                    self.motors_wait_thread.prev_speed[:] = [0] * 5

                for name in motors_names:
                    if args[0] == name and args[1]:
                        motors_speed[motors_names.index(name)] = int(args[1])
                        print("Changing {} speed to {}".format(name, int(args[1])))

                if args[0] == 'h':
                    motors_speed[0] = int(args[1])
                    motors_speed[1] = int(args[1])
                    print("Changing H motors speeds to {}".format(int(args[1])))

                if args[0] == 'v':
                    motors_speed[2] = int(args[1])
                    motors_speed[3] = int(args[1])
                    motors_speed[4] = int(args[1])
                    print("Changing V motors speeds to {}".format(int(args[1])))

                if args[0] == 'vlr':
                    motors_speed[2] = int(args[1])
                    motors_speed[4] = int(args[1])
                    print("Changing V l&r motors speeds to {}".format(int(args[1])))

                if args[0] == "pid":
                    var = args[1]
                    Kp, Ki, Kd = float(args[2]), float(args[3]), float(args[4])
                    if var == 'x':
                        pid_thread.roll_PID.setPIDCoefficients(Kp, Ki, Kd)

                    if var == 'y':
                        pid_thread.pitch_PID.setPIDCoefficients(Kp, Ki, Kd)

                    if var == 'z':
                        pid_thread.yaw_PID.setPIDCoefficients(Kp, Ki, Kd)

                    if var == 'v':
                        pid_thread.velocity_PID.setPIDCoefficients(Kp, Ki, Kd)

                if args[0] == "vel":
                    pid_thread.velocity_PID.setSetPoint(args[1])

                if args[0] == "run":
                    run_flag = True
                if args[0] == 'stop':
                    run_flag = False

                #elif len(args) != 3:
                    #motor_wait_thread = MotorsWaitThread(float(args[2]), self.prev_speed)
                    #motor_wait_thread.start()


class PadSteeringThread(threading.Thread):
    """Thread class that is terminal's UI for setting PIDs, primitive steering by commands """
    def __init__(self, pid_thread):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.pid_thread = pid_thread
        self.last_time = time.time()

        # stworzenie wątków do odczytu danych z pada
        self.connection = Connection(IP_ADDRESS_2, PAD_PORT)
        self.read_values_thread = threading.Thread(target=self.connection)


        # strojenie regulatorow #hardcoded
        pid_thread.roll_PID.setPIDCoefficients(4, 2, 2) # zahardkodowane narazie # nastawy  z testow
        pid_thread.pitch_PID.setPIDCoefficients(10, 2, 1)
        pid_thread.yaw_PID.setPIDCoefficients(0, 0, 0) # zera, bo horyzontalnymi silnikami sterujemy tylko padem

    def run(self):
        global motors_speed, motors_speed_pad, run_flag
        self.connection.start()
        while True:
            data_frame = self.connection.getDataFrame()
            with self.lock:
                if len(data_frame) == 5:
                    motor_0_duty = data_frame[0]
                    motor_1_duty = data_frame[1]
                    roll_offset = data_frame[2]
                    pitch_offset = data_frame[3]
                    #depth_offset = data_frame[4]
                    vertical_duty = data_frame[4] # bez glebokosciomierza

                    motors_speed_pad[0] = (-0.5)*motor_0_duty
                    motors_speed_pad[1] = (-0.5)*motor_1_duty
                    motors_speed_pad[2] = 0.3*vertical_duty
                    motors_speed_pad[3] = 0.3*vertical_duty
                    motors_speed_pad[4] = 0.3*vertical_duty
                    self.pid_thread.roll_PID.setSetPoint(roll_offset)
                    self.pid_thread.pitch_PID.setSetPoint(pitch_offset)
                    if SAVE_FLAG:
                        self.save(data_frame)
                    # print(data_frame)
                    #self.pid_thread.depth_PID.setSetPoint(depth_offset)  # can't use without depth funcionalities

    def save(self, data_frame):
        with open(MOVES_FILE, 'ab') as file:
            data_frame.append(time.time()-self.last_time)
            self.last_time = time.time()
            file.write(pickle.dumps(data_frame))


class ReadSteeringThread(threading.Thread):
    """Thread class that play primitive steering by commands from file """
    def __init__(self, pid_thread):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.pid_thread = pid_thread
        self.file = None

        # strojenie regulatorow #hardcoded
        pid_thread.roll_PID.setPIDCoefficients(4, 2, 2) # zahardkodowane narazie # nastawy  z testow
        pid_thread.pitch_PID.setPIDCoefficients(10, 2, 1)
        pid_thread.yaw_PID.setPIDCoefficients(0, 0, 0) # zera, bo horyzontalnymi silnikami sterujemy tylko plikiem

    def run(self):
        global motors_speed, run_flag, motors_speed_pad
        self.file = open(MOVES_FILE, "rb")
        while True:
            try:
                data_frame = pickle.load(self.file)
                #print(data_frame)
            except(EOFError, pickle.UnpicklingError):
                with self.lock:
                    self.pid_thread.pid_motors_speeds_update[0] = 0
                    self.pid_thread.pid_motors_speeds_update[1] = 0
                    self.pid_thread.pid_motors_speeds_update[2] = 0
                    self.pid_thread.pid_motors_speeds_update[3] = 0
                    self.pid_thread.pid_motors_speeds_update[4] = 0
                    #print("KONIEC")
                    break

            with self.lock:
               if len(data_frame) == 6:
                    motor_0_duty = data_frame[0]
                    motor_1_duty = data_frame[1]
                    roll_offset = data_frame[2]
                    pitch_offset = data_frame[3]
                    #depth_offset = data_frame[4]
                    vertical_duty = data_frame[4] # bez glebokosciomierza

                    time.sleep(data_frame[5]) # usypiamy na dany czas

                    motors_speed_pad[0] = (-0.5)*motor_0_duty
                    motors_speed_pad[1] = (-0.5)*motor_1_duty
                    motors_speed_pad[2] = 0.3*vertical_duty
                    motors_speed_pad[3] = 0.3*vertical_duty
                    motors_speed_pad[4] = 0.3*vertical_duty
                    self.pid_thread.roll_PID.setSetPoint(roll_offset)
                    self.pid_thread.pitch_PID.setSetPoint(pitch_offset)
                    print(data_frame)
                    #self.pid_thread.depth_PID.setSetPoint(depth_offset)  # can't use without depth funcionalities


class AutonomyTest(threading.Thread):
    """Thread class that is primitive autonomy"""
    def __init__(self, pid_thread):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.pid_thread = pid_thread
        self.last_time = time.time()

        global run_flag

        # strojenie regulatorow #hardcoded
        self.pid_thread.roll_PID.setPIDCoefficients(4, 2, 2) # zahardkodowane narazie # nastawy  z testow
        self.pid_thread.pitch_PID.setPIDCoefficients(10, 2, 1)
        self.pid_thread.yaw_PID.setPIDCoefficients(0, 0, 0) # zera, bo horyzontalnymi silnikami sterujemy tylko padem
        self.pid_thread.center_x_PID.setPIDCoefficients(0, 0, 0)
        self.pid_thread.center_y_PID.setPIDCoefficients(0, 0, 0)

        self.server = Server(IP_ADDRESS_2, PORT)

    def run(self):
        while True:
            date_frame = self.server.receiveData()
            if len(date_frame[0]):
                with self.lock:
                    self.pid_thread.center_y_diff = self.pid_thread.center_y_PID.update(date_frame[0][])
                    self.pid_thread.center_x_diff = self.pid_thread.center_x_PID.update(date_frame[0][])
                if date_frame:
                    run_flag = True
                else:
                    run_flag = False





#motors_control_thread = MotorsControlThread()
imu = IMUClass('roll', 'pitch', 'yaw', 'accel_proc_x', 'accel_proc_z', 'accel_proc_y','gyro_proc_z' ,'gyro_proc_x', 'gyro_raw_z')

#connThread = Connection(IP_ADDRESS)
#dht_thread = DHTThread()
motors_control_thread = MotorsControlThread()

imu_thread = IMUThread(imu)
#imu_thread.setConnection(connThread)

pid_thread = PIDThread()
pid_thread.setIMU(imu)
ui_thread = UIThread(pid_thread)
if PAD_STEERING_FLAG:
    pad_steering_thread = PadSteeringThread(pid_thread)
if READ_FLAG:
    read_steering_flag = ReadSteeringThread(pid_thread)


# opening another terminal and executing output.log tailing
# os.system("gnome-terminal -e 'tail -f output.log'")   # works on PC Ubuntu
# os.system("mate-terminal --window --working-directory='~/autonomous-underwater-vehicle/Odroid/main-tests' --command='tailOutput.sh'")
# os.system("sh -c '~/autonomous-underwater-vehicle/Odroid/main-tests/tailOutput.sh'")

imu_thread.start()
#dht_thread.start()
#connThread.start()
motors_control_thread.start()
pid_thread.start()
if PAD_STEERING_FLAG:
    pad_steering_thread.start()
if READ_FLAG:
    read_steering_flag.start()

ui_thread.start()
