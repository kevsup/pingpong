import RPi.GPIO as GPIO
import threading
import time
import atexit
from collections import deque
from pingpong_constants import *


GPIO.setmode(GPIO.BOARD)

t0 = time.time()
shootingTime = 3000 # milliseconds

state = STATES['WAIT_FOR_USER']
shootingState = STATES['FEED']
shootingSequence = DEFAULT_SEQUENCE
sequencePtr = 0
nextShot = shootingSequence[sequencePtr]

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

prevA = True
prevB = True
encoderTicks = 0
thetaMotor = encoderTicks * RADS_PER_TICK
thetaMotorQ = deque()
omegaMotorQ = deque()
prevTime = time.time()

thetaDes = 0
reachedDes = False

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

def setThetaDes():
    global thetaDes
    global reachedDes
    reachedDes = False
    if nextShot == STATES['FOREHAND']:
        thetaDes = THETADES['FOREHAND']
    else:
        thetaDes = THETADES['BACKHAND']

def setStateToSweep():
    global state
    state = STATES['SWEEP']
    setThetaDes()
    printState(state)

def shoot():
    global shootingState
    global state
    global t0
    global nextShot
    global sequencePtr
    if shootingState == STATES['FEED']:
        servo.ChangeDutyCycle(DEG180)
        shootingState = STATES['WAIT']
        t0 = time.time()
    elif shootingState == STATES['WAIT']:
        currTime = (int)(1000 * (time.time() - t0))
        if currTime > shootingTime:
            shootingState = STATES['FEED']
            updateSpin()
            sequencePtr = sequencePtr + 1
            if sequencePtr >= len(shootingSequence):
                sequencePtr = 0
            nextShot = shootingSequence[sequencePtr]
            printState(nextShot)
            if nextShot != state:
                setStateToSweep()
        elif currTime > shootingTime / 2:
            servo.ChangeDutyCycle(DEG0)
    else:
        print('yo shooting state cannot be stateless!')

def getQueueAvg(queue):
    return 1.0 * sum(queue) / len(queue)

def updateMovingQueue(queue, val):
    queue.popleft()
    queue.append(val)

def checkEncoder(encoder):
    global prevA
    global prevB
    global encoderTicks
    global thetaMotor
    global prevTime
    currA = GPIO.input(PINS['ENCODER_A'])
    currB = GPIO.input(PINS['ENCODER_B'])
    if (encoder=='A' and currA==prevA) or \
            (encoder=='B' and currB==prevB):
        return False
    if currA == currB:
        if encoder == 'A':
            encoderTicks = encoderTicks + 1
        else:
            encoderTicks = encoderTicks - 1
    else:
        if encoder == 'A':
            encoderTicks = encoderTicks - 1
        else:
            encoderTicks = encoderTicks + 1
    prevA = currA
    prevB = currB
    currTime = time.time()
    elapsedTime = currTime - prevTime
    prevTime = currTime
    prevThetaMotor = thetaMotor
    thetaMotor = encoderTicks * RADS_PER_TICK
    omegaMotor = (thetaMotor - prevThetaMotor) / elapsedTime
    updateMovingQueue(thetaMotorQ, thetaMotor)
    updateMovingQueue(omegaMotorQ, omegaMotor)
    print('theta = ' + str(getQueueAvg(thetaMotorQ)) + \
           ', omega = ' + str(getQueueAvg(omegaMotorQ)))
    return True

def feedbackControl(thetaMotorDes):
    voltage = kp * (thetaMotorDes - getQueueAvg(thetaMotorQ)) + \
            kd * getQueueAvg(omegaMotorQ)
    if voltage > MAX_VOLTAGE:
        voltage = MAX_VOLTAGE
    elif voltage < -MAX_VOLTAGE:
        voltage = -MAX_VOLTAGE
    if voltage > 0:
        GPIO.output(PINS['GEAR_DIR_A'], COUNTERCLOCKWISE['GEAR_DIR_A'])
        GPIO.output(PINS['GEAR_DIR_B'], COUNTERCLOCKWISE['GEAR_DIR_B'])
    else:
        GPIO.output(PINS['GEAR_DIR_A'], CLOCKWISE['GEAR_DIR_A'])
        GPIO.output(PINS['GEAR_DIR_B'], CLOCKWISE['GEAR_DIR_B'])
    dutyCycle = 100.0 * abs(voltage) / MAX_VOLTAGE
    gearMotor.ChangeDutyCycle(dutyCycle)

def sweep():
    global t0
    global reachedDes
    if not checkEncoder('A'):
        checkEncoder('B')
    error = thetaDes * THETADES_TO_THETAMOTOR - getQueueAvg(thetaMotorQ)
    if abs(error) < 1 and not reachedDes:
        reachedDes = True
        t0 = time.time()
    if (not reachedDes) or (time.time() - t0 < SETTLING_TIME):
        feedbackControl(thetaDes * THETADES_TO_THETAMOTOR)
        return False
    else:
        gearMotor.ChangeDutyCycle(0)
        return True

def setup():
    inputPins = ['ENCODER_A', 'ENCODER_B']
    for pin in PINS:
        if pin not in inputPins and pin not in PWM_PINS:
            GPIO.setup(PINS[pin], GPIO.OUT)
        elif pin not in PWM_PINS:
            GPIO.setup(PINS[pin], GPIO.IN, pull_up_down = GPIO.PUD_UP)
    servo.start(DEG0)
    setupMotorDir()

    #temp until implement flask
    setStateToSweep()

    global prevA
    global prevB
    prevA = GPIO.input(PINS['ENCODER_A'])
    prevB = GPIO.input(PINS['ENCODER_B'])
    
    #setup moving average queues
    for i in range(LEN_MOVING_AVERAGE):
        thetaMotorQ.append(0)
        omegaMotorQ.append(0)

def finiteStateMachine():
    global state
    global sequencePtr
    global nextShot
    try:
        while True:
            if state == STATES['SWEEP']:
                if sweep():
                    # done sweeping, go to shooting
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

