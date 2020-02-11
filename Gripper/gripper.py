import wiringpi

wiringpi.wiringPiSetup()
#sterownik silnika
en_IN1_Pin=15 #clockwise
en_IN2_Pin=16 #counter-clockwise


class Gripper:
    def __init__(self):
        wiringpi.pinMode(en_IN1_Pin,1)
        wiringpi.pinMode(en_IN2_Pin,1)
        self.dir=0
        self.run=0
    def move_clockwise(self):
        wiringpi.digitalWrite(en_IN1_Pin,1)
        wiringpi.digitalWrite(en_IN2_Pin,0)
        self.dir=0
        self.run=1
    def move_counter_clockwise(self):
        wiringpi.digitalWrite(en_IN1_Pin,0)
        wiringpi.digitalWrite(en_IN2_Pin,1)
        self.dir=1
        self.run=1
    def stop(self):
        wiringpi.digitalWrite(en_IN1_Pin,0)
        wiringpi.digitalWrite(en_IN2_Pin,0)
        self.dir=0
        self.run=1
    def check_dir(self):
        return self.dir
    def check_run(self):
        return self.run

if __name__=="__main__":
    gripper=Gripper()
    gripper.move_clockwise()




