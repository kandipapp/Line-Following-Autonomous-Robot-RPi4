import RPi.GPIO as GPIO
import time
from time import sleep
import sys 
import math 

enA = 2
enB = 22
in1 = 3
in2 = 4
in3 = 17
in4 = 27
rotR= 5
rotL= 6

#setup 
GPIO.setmode(GPIO.BCM)

#setup Motor Driver 
GPIO.setup(enA,GPIO.OUT)
GPIO.setup(enB,GPIO.OUT)

GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)

GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)

RIGHT = GPIO.PWM(enA,1000) #frequency = 1000Hz 
LEFT = GPIO.PWM(enB,1000)

RIGHT.start(0)
LEFT.start(0)

GPIO.setup(rotR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #rotary
GPIO.setup(rotL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

pulseCountL = 0
pulseCountR = 0
offCountR = 0
offCountL = 0
eqDistL = 0
eqDistR = 0
wheelCircumference = 2 * 3 * 3; # Circumference (radius = 3 cm)

def forward(speed):
    RIGHT.ChangeDutyCycle(speed)
    LEFT.ChangeDutyCycle(speed)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def backward(speed):
    RIGHT.ChangeDutyCycle(speed)
    LEFT.ChangeDutyCycle(speed)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def stop():
    RIGHT.ChangeDutyCycle(0)
    LEFT.ChangeDutyCycle(0)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

# Define turning functions with PWM and time
turn_values = {
    15: (65, 0.25),
    30: (69, 0.3),
    45: (68, 0.4),
    90: (70, 0.75),
    180: (73, 1.3),
    360: (100, 1.63)
}

def right_turn(pwm, duration):
    RIGHT.ChangeDutyCycle(pwm)
    LEFT.ChangeDutyCycle(pwm)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    sleep(duration)
    stop()

def left_turn(pwm, duration):
    RIGHT.ChangeDutyCycle(pwm)
    LEFT.ChangeDutyCycle(pwm)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    sleep(duration)
    stop()

def execute_turn(degree, direction):
    available_turns = sorted(turn_values.keys(), reverse=True)
    best_combo = []
    while degree > 0:
        best_match = min(available_turns, key=lambda x: abs(x - degree))
        if best_match == 0:
            break
        best_combo.append(best_match)
        degree -= best_match
    for turn in best_combo:
        pwm, duration = turn_values[turn]
        if direction == 'r':
            right_turn(pwm, duration)
        elif direction == 'l':
            left_turn(pwm, duration)

def countPulse():
    global pulseCountR, pulseCountL, eqDistR, eqDistL, offCountR, offCountL
    onCountR = GPIO.input(rotR)
    onCountL = GPIO.input(rotL)

    if offCountR != onCountR:
        pulseCountR += 1
        eqDistR = (pulseCountR*wheelCircumference/40)
        offCountR = onCountR

    if offCountL != onCountL:
        pulseCountL += 1
        eqDistL =(pulseCountL*wheelCircumference/40)
        offCountL = onCountL


def opt2():
    global eqDistL,pulseCountL
    eqDistL = 0
    pulseCountL = 0
    
    dist = float(input("Enter diastance you wish to travel: ").strip())
    speed = int(input("Enter duty Cycle: ").strip())
    
    backward(speed)
    while eqDistL < dist:
        countPulse()
        print("Current Distance: ", eqDistL)
        time.sleep(0.1)
        
    stop()
    print("Target distance reached!")

def opt1(): 
    try:
        while True:
            direction = input("Enter Command (f/b/r/l/s/e): ").strip().lower()
            if direction in ['f', 'b']:
                speed = int(input("Enter duty Cycle: ").strip())
                if direction == 'f':
                    forward(speed)
                else:
                    backward(speed)
                sleep(2)
                stop()
            elif direction in ['r', 'l']:
                degree = int(input("Enter degree of turn: ").strip())
                execute_turn(degree, direction)
            elif direction == 's':
                stop()
                main()
            elif direction == 'e':
                GPIO.cleanup()
                break
            else:
                print("Invalid input! Use f/b/r/l/s/e")
    except KeyboardInterrupt:
        print("\nProgram interrupted. Cleaning up GPIO...")
        GPIO.cleanup()

def main():
    while True: 
        print("Welcome to my world!!!")
        print("Option 1: Angle measurement")
        print("Option 2: Distance measurement")
        print("Option 3: Return to menu")
        
        choice = int(input("Enter (1 / 2 /3) to measure angle or distance: "))
        
        if choice == 1:
            opt1()
            main()
        elif choice == 2:
            opt2()
            main()
        elif choice == 3:
            GPIO.cleanup()
            print("Exiting...")
            break
        else:
            print("Invalud choice! Please enter 1,2, or 3.")
            
main()
