import wiringpi


#wykrywanie dna
sensorL_Pin=4
sensorR_Pin=5

#czujniki wewnatrz chwytaka
sensorI_Pins=[6,7,8,9]

class SensorSystem:
    def __init__(self):
        wiringpi.pinMode(0, sensorL_Pin)
        wiringpi.pinMode(0, sensorR_Pin)
        self.sensorR_state=0
        self.sensorL_state=0
        self.sensorI_states=[]
        for pin in sensorI_Pins:
            wiringpi.pinMode(0,pin)
        for i in range(len(sensorI_Pins)):
            self.sensorI_states[i]=0
        self.sensorI_state=0
        self.sensorRL_state=0

    def check_sensorI(self):
        for i in range(len(sensorI_Pins)):
            state=wiringpi.digitalRead(sensorI_Pins[i])
            self.sensorI_states[i]=state
            if state == 1:
                self.sensorI_state=1
        return self.sensorI_state

    def check_sensorRL(self):
        self.sensorL_state=wiringpi.digitalRead(sensorL_Pin)
        self.sensorR_state=wiringpi.digitalRead(sensorR_Pin)
        self.sensorRL_state=self.sensorL_state and self.sensorR_state
        return self.sensorRL_state

    def read_sensorI(self):
        return self.sensorI_state

    def read_sensorRL(self):
        return self.sensorRL_state
