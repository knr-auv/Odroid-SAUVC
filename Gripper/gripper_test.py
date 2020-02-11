import time

from gripper import Gripper
from sensor_system import SensorSystem

gripper=Gripper()
sensors=SensorSystem()
gripper.move_clockwise()
print("moving")
while True:
    if gripper.check_dir()==0:
        if sensors.detect_bottom():
            print("bottom")
            gripper.stop()
            time.sleep(2)
            if sensors.detect_ball():
                gripper.move_counter_clockwise()
                print("ball caught")
        if sensors.detect_max_pos_d():
            print("maxd")
            gripper.move_counter_clockwise()
    else:
        if sensors.detect_max_pos_u():
            print("maxu")
            gripper.stop()
            time.sleep(3)
            gripper.move_clockwise()
            print("moving")
    time.sleep(0.01)
