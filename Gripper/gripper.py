import logging
import time

import wiringpi

NFAULT_PIN = 1
STEP_PIN = 2
DIR_PIN = 3
NENBL_PIN = 4
MS1_PIN = 5
MS2_PIN = 6
NSLEEP_PIN = 7
PULSE_TIME = 1  # us


class Gripper:
    def __init__(self, step_mode= 1, dir = 0):
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(NFAULT_PIN, 0)
        wiringpi.pinMode(STEP_PIN, 1)
        wiringpi.pinMode(DIR_PIN, 1)
        wiringpi.pinMode(NENBL_PIN, 1)
        wiringpi.pinMode(MS1_PIN, 1)
        wiringpi.pinMode(MS2_PIN, 1)
        wiringpi.pinMode(NSLEEP_PIN, 1)
        self.set_step_mode(step_mode)
        self.set_dir(dir)
        self.dir = dir
        self.disable()
        self.en = False
        self.sleep()
        self.sleep_flag = True
        self.fault = False
        self.pulse_time = PULSE_TIME * 0.000001

    def set_step_mode(self, step_mode):
        # step_mode values:
        # 1 - full step
        # 2 - half step
        # 3 - quarter step
        # 4 - eighth step
        if step_mode == 1 or step_mode == 2 or step_mode == 3 or step_mode == 4:
            if step_mode == 1:
                wiringpi.digitalWrite(MS2_PIN, 0)
                wiringpi.digitalWrite(MS1_PIN, 0)
                logging.debug('MS2 = 0, MS1 = 0')
            elif step_mode == 2:
                wiringpi.digitalWrite(MS2_PIN, 0)
                wiringpi.digitalWrite(MS1_PIN, 1)
                logging.debug('MS2 = 0, MS1 = 1')
            elif step_mode == 3:
                wiringpi.digitalWrite(MS2_PIN, 1)
                wiringpi.digitalWrite(MS1_PIN, 0)
                logging.debug('MS2 = 1, MS1 = 0')
            else:
                wiringpi.digitalWrite(MS2_PIN, 1)
                wiringpi.digitalWrite(MS1_PIN, 1)
                logging.debug('MS2 = 1, MS1 = 1')
            self.step_mode = step_mode
        else:
            logging.debug('invalid step_mode')

    def set_dir(self, dir):
        # dir values:
        # 0 - backwards
        # 1 - forwards
        if dir == 0 or dir == 1:
            wiringpi.digitalWrite(DIR_PIN, dir)
            self.dir = dir
            logging.debug("dir = %d", dir)
        else:
            logging.debug('invalid dir')

    def enable(self):
        if self.sleep_flag == True:
            self.wake()
        wiringpi.digitalWrite(NENBL_PIN, 0)
        logging.debug('nENBL = 0')
        self.en = True

    def disable(self):
        wiringpi.digitalWrite(NENBL_PIN, 1)
        logging.debug('nENBL = 1')
        self.en = False

    def sleep(self):
        wiringpi.digitalWrite(NSLEEP_PIN, 0)
        logging.debug('nSLEEP = 0')
        self.sleep_flag = True

    def wake(self):
        wiringpi.digitalWrite(NSLEEP_PIN, 1)
        logging.debug('nSLEEP = 1')
        self.sleep_flag = False

    def fault_detect(self):
        fault = not bool(wiringpi.digitalRead(NFAULT_PIN))
        # fault = not bool(1)
        if fault == True:
            self.fault = True
        else:
            self.fault = False

    def get_fault(self):
        return self.fault

    def step(self):
        if not self.sleep_flag:
            if self.en:
                self.fault_detect()
                if not self.fault:
                    wiringpi.digitalWrite(STEP_PIN, 1)
                    logging.debug('STEP = 1')
                    time.sleep(self.pulse_time)
                    wiringpi.digitalWrite(STEP_PIN, 0)
                    logging.debug('STEP = 0')
                else:
                    logging.debug('fault')
            else:
                logging.debug('stepper motor disabled')
        else:
            logging.debug('sleep mode is on')


if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)
    gripper = Gripper(1, 1)
    gripper.enable()
    # while gripper.get_fault()==False:
    for i in range(10):
        gripper.step()
