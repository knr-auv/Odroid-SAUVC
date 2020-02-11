import wiringpi


class SensorSystem:
    def __init__(self):
        # self.ball_sensors_pins=[6,3,4,5]
        #self.ball_sensors_pins = [6]
        self.ball_sensors_pins = [4,6]
        self.max_pos_u_pin = 26
        self.max_pos_d_pin = 27
        # self.bottom_det_pins = [1, 4]
        self.bottom_det_pins = [1]
        wiringpi.wiringPiSetup()
        for pin in self.ball_sensors_pins:
            wiringpi.pinMode(pin, 0)
            wiringpi.pullUpDnControl(pin, 1)  # pullup
        wiringpi.pinMode(self.max_pos_u_pin, 0)
        wiringpi.pullUpDnControl(self.max_pos_u_pin, 1)
        wiringpi.pinMode(self.max_pos_d_pin, 0)
        wiringpi.pullUpDnControl(self.max_pos_d_pin, 1)
        for pin in self.bottom_det_pins:
            wiringpi.pinMode(pin, 0)
            wiringpi.pullUpDnControl(pin, 1)

    def detect_ball(self):
        state = 0
        for pin in self.ball_sensors_pins:
            state = state or (wiringpi.digitalRead(pin))
        return state

    def detect_bottom(self):
        state = 1
        for pin in self.bottom_det_pins:
            state = state and (not wiringpi.digitalRead(pin))
        return state

    def detect_max_pos_u(self):
        state = not wiringpi.digitalRead(self.max_pos_u_pin)
        return state

    def detect_max_pos_d(self):
        state = not wiringpi.digitalRead(self.max_pos_d_pin)
        return state
