import RPi.GPIO as GPIO
import threading
import time
import atexit
from pingpong_constants import *

GPIO.setmode(GPIO.BOARD)

t0 = time.time()
shootingTime = 3000 # milliseconds

state = STATES['WAIT_FOR_USER']
nextShot = STATES['BACKHAND']
shootingState = STATES['FEED']

spin = SPINS['BACKSPIN']

# set up PWM pins
GPIO.setup(PINS['SERVO'], GPIO.OUT) 
servo = GPIO.PWM(PINS['SERVO'], SERVO_PWM_FREQ)    
servo.start(0)

GPIO.setup(PINS['MB_PWM'], GPIO.OUT)
bottomMotor = GPIO.PWM(PINS['MB_PWM'], PWM_FREQ)
bottomMotor.start(0)

GPIO.setup(PINS['MT_PWM'], GPIO.OUT)
topMotor = GPIO.PWM(PINS['MT_PWM'], PWM_FREQ)
topMotor.start(0)

GPIO.setup(PINS['GEAR_PWM'], GPIO.OUT)
gearMotor = GPIO.PWM(PINS['GEAR_PWM'], PWM_FREQ)
gearMotor.start(0)

# deal with encoder later

def setupMotorDir():
    GPIO.output(PINS['MB_DIR_A'], GPIO.LOW)
    GPIO.output(PINS['MB_DIR_B'], GPIO.HIGH)
    GPIO.output(PINS['MT_DIR_A'], GPIO.HIGH)
    GPIO.output(PINS['MT_DIR_B'], GPIO.LOW)

def updateSpin():
    if spin == SPINS['TOPSPIN']:
        bottomMotor.ChangeDutyCycle(TOPSPIN_PWM['bottom'])
        topMotor.ChangeDutyCycle(TOPSPIN_PWM['top'])
    else:
        bottomMotor.ChangeDutyCycle(BACKSPIN_PWM['bottom'])
        topMotor.ChangeDutyCycle(BACKSPIN_PWM['top'])

def printState(stateNum):
    for s in STATES:
        if STATES[s] == stateNum:
            print(s)

def shoot():
    global shootingState
    global state
    global t0
    if shootingState == STATES['FEED']:
        servo.ChangeDutyCycle(DEG180)
        shootingState = STATES['WAIT']
        t0 = time.time()
    elif shootingState == STATES['WAIT']:
        currTime = (int)(1000 * (time.time() - t0))
        if currTime > shootingTime:
            shootingState = STATES['FEED']
            updateSpin()
            if nextShot != state:
                state = STATES['SWEEP'] 
                printState(state)
        elif currTime > shootingTime / 2:
            servo.ChangeDutyCycle(DEG0)
    else:
        print('yo shooting state cannot be stateless!')

def setup():
    inputPins = ['ENCODER_A', 'ENCODER_B']
    for pin in PINS:
        if pin not in inputPins and pin not in PWM_PINS:
            GPIO.setup(PINS[pin], GPIO.OUT)
        elif pin not in PWM_PINS:
            GPIO.setup(PINS[pin], GPIO.IN)
    servo.start(DEG0)
    setupMotorDir()

    #temp until implement flask
    global state
    state = STATES['SWEEP']

def finiteStateMachine():
    global state
    try:
        while True:
            if state == STATES['SWEEP']:
                print('do later')

                # done sweeping, go to feed
                state = nextShot
                updateSpin()
                printState(state)
            elif state == STATES['FOREHAND'] or state == STATES['BACKHAND']:
                shoot()
            elif state != STATES['WAIT_FOR_USER']:
                print('yo main state cannot be stateless!')
    except KeyboardInterrupt:
        print('keyboard exit detected')



setup()
finiteStateMachine()
atexit.register(GPIO.cleanup)

