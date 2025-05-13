import RPi.GPIO as GPIO
import time

enA = 2
enB = 22
in1 = 3
in2 = 4
in3 = 17
in4 = 27
rotR = 5
rotL = 6

GPIO.setmode(GPIO.BCM)
GPIO.setup(enA,GPIO.OUT) #Motor L298N
GPIO.setup(enB,GPIO.OUT)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)

A=GPIO.PWM(enA,1000)
B=GPIO.PWM(enB,1000)

A.start(0)
B.start(0)

GPIO.setup(rotR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #rotary
GPIO.setup(rotL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

pulseCountL = 0
pulseCountR = 0
offCountR = 0
offCountL = 0
eqDistL = 0
eqDistR = 0
wheelCircumference = 2 * 3 * 3; # Circumference (radius = 3 cm)

def forward():
    A.ChangeDutyCycle(50)
    B.ChangeDutyCycle(50)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
 
def backward():
    A.ChangeDutyCycle(50)
    B.ChangeDutyCycle(50)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    
def right():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

def left():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    
def stop():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)

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
    
    forward()
    while eqDistL < dist:
        countPulse()
        print("Current Distance: ", eqDistL)
        time.sleep(0.1)
        
    stop()
    print("Target distance reached!")

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
        
def opt1():
    try:
        while True:
            x = input("Enter command (f/b/r/l/s/e): ").strip().lower()
            

            print("The distance of Right wheel is", eqDistR)
            print("The distance of Left wheel is", eqDistL)  # Fixed typo

            if x == 'f':
                forward()
            elif x == 'b':
                backward()
            elif x == 'r':
                A.ChangeDutyCycle(25)
                B.ChangeDutyCycle(25)
                right()
            elif x == 'l':
                A.ChangeDutyCycle(25)
                B.ChangeDutyCycle(25)
                left()
            elif x == 's':  # Stop command
                stop()
            elif x == 'e':  # Exit the program
                GPIO.cleanup()
                break
            else:
                print("Invalid input! Use f/b/r/l/s/e")
                
            for _ in range(10):
                countPulse
                print("Right Wheel: ", eqDistR, "cm")
                print("Left Wheel: ", eqDistL, "cm")
                time.sleep(0.1) 

    except KeyboardInterrupt:
        print("\nExiting program...")

    finally:    
        GPIO.cleanup()
        
main() 