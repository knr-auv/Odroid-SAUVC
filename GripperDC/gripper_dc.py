import wiringpi


wiringpi.wiringPiSetup()

#sterownik silnika
en_L_PWM_Pin=0
en_R_PWM_Pin=1
en_L_enable_Pin=2
en_R_enable_Pin=3

en_PWM_Pin=4
en_dir_Pin=5

class Gripper:
    def __init__(self):

        wiringpi.pinMode(1, en_L_enable_Pin)
        wiringpi.pinMode(1, en_R_enable_Pin)
        wiringpi.pinMode(1, en_L_PWM_Pin)
        wiringpi.pinMode(1, en_R_PWM_Pin)

        wiringpi.softPwmCreate(en_L_PWM_Pin, 0, 100)
        wiringpi.softPwmCreate(en_R_PWM_Pin, 0, 100)
        self.dir=0

    def engineL_writePWM(self, duty):
        self.engineR_disable()
        self.engineL_enable()
        if duty<=100 and duty>=0:
            wiringpi.softPwmWrite(en_L_enable_Pin, duty)

    def engineR_writePWM(self, duty):
        self.engineL_disable()
        self.engineR_enable()
        if duty<=100 and duty>=0:
            wiringpi.softPwmWrite(en_R_enable_Pin, duty)

    def engineL_enable(self):
        wiringpi.digitalWrite(en_L_enable_Pin, 1)

    def engineL_disable(self):
        wiringpi.digitalWrite(en_L_enable_Pin, 0)

    def engineR_enable(self):
        wiringpi.digitalWrite(en_R_enable_Pin,1)

    def engineR_disable(self):
        wiringpi.digitalWrite(en_R_enable_Pin,0)






