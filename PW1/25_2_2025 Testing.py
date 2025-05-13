import RPi.GPIO as GPIO
import time

GPIO.cleanup()
testPIN = 26



GPIO.setmode(GPIO.BCM)
GPIO.setup(testPIN,GPIO.OUT)


while True: 
    print("Starting motors at 50% speed...")
    GPIO.output(testPIN,GPIO.HIGH)



