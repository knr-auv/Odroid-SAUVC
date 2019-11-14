import logging
from gripper import Gripper
from threading import Thread, Lock
import time

class GripperController:
    def __init__(self):
        self.task = 0
        self.lock = Lock()
        self.gripper = Gripper(1,1)

    def set_task(self, task):
        with self.lock:
            self.task = task
    def enable(self):
        self.gripper.enable()

    def set_step_mode(self, step_mode):
        self.gripper.set_step_mode(step_mode)

    def disable(self):
        self.gripper.disable()

    def sleep(self):
        self.gripper.sleep()

    def wake(self):
        self.gripper.wake()

    def get_fault(self):
        return self.gripper.fault

    def step(self):
        self.gripper.step()
    def down(self):
        self.set_task(1)
        t=TaskPerformer(self)
        t.start()
    def up(self):
        self.set_task(2)
        t=TaskPerformer(self)
        t.start()
    def stop(self):
        self.set_task(0)


class TaskPerformer(Thread):
    def __init__(self, gripper_controller):
        Thread.__init__(self)
        self.gripper = gripper_controller.gripper
        self.gripper_controller = gripper_controller
        self.task = gripper_controller.task

    def run(self):
        while self.task == self.gripper_controller.task:
            if self.task == 1:
                if self.gripper.dir == 0:
                    self.gripper.set_dir(1)
                if self.gripper.get_fault() == False:
                    
                    self.gripper.step()
                else:
                    self.gripper_controller.set_task(0)
                    break
            elif self.task == 2:
                if self.gripper.dir == 1:
                    self.gripper.set_dir(0)
                if self.gripper.get_fault() == False:
                    self.gripper.step()
                else:
                    self.gripper_controller.set_task(0)
                    break
            time.sleep(0.000001)


class InputListener(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.gripper_controller = GripperController()
        self.gripper_controller.enable()

    def run(self):
        while True:
            s = input()
            try:
                if s[0] == 'd':
                    self.gripper_controller.down()
                elif s[0] == 'u':
                    self.gripper_controller.up()
                elif s[0] == 's':
                    self.gripper_controller.stop()
                elif s[0] == 'm':
                    m=int(s[2])
                    if m == 1 or m==2 or m==3 or m==4:
                        self.gripper_controller.set_step_mode(m)
                elif s[0] == 'k':
                    self.gripper_controller.disable()
                    break
            except Exception as e:
                logging.warning(e)
            finally:
                time.sleep(0.01)



if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)
    input_listener = InputListener()
    input_listener.start()
