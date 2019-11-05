import wiringpi
import DHT
import time
import datetime

#print("gunwo")
instance = DHT.DHT11(6)
#print("shieeeeet")
try:
    while True:
        result = instance.read()
        if result.is_valid():
            print("Last valid input: " + str(datetime.datetime.now()))
            print("Temperature: %-3.1f C" % result.temperature)
            print("Humidity: %-3.1f %%" % result.humidity)
        #else: print("oneoneone")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Cleanup")
    GPIO.cleanup()
