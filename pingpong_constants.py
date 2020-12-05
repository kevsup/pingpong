import RPi.GPIO as GPIO

PINS = {}
PINS['MB_DIR_A'] = 13
PINS['MB_DIR_B'] = 11
PINS['MB_PWM'] = 15
PINS['MT_PWM'] = 16
PINS['MT_DIR_A'] = 18
PINS['MT_DIR_B'] = 22
PINS['GEAR_PWM'] = 37
PINS['GEAR_DIR_A'] = 32
PINS['GEAR_DIR_B'] = 36
PINS['ENCODER_A'] = 31
PINS['ENCODER_B'] = 33
PINS['SERVO'] = 29

DEG0 = 2.5  # 0 degrees servo PWM
DEG180 = 12.5   # 180 degrees servo PWM

STATES = {}
STATES['WAIT_FOR_USER'] = 0
STATES['SWEEP'] = 1
STATES['FEED'] = 2
STATES['WAIT'] = 3
STATES['FOREHAND'] = 4
STATES['BACKHAND'] = 5

DEFAULT_SEQUENCE = [STATES['FOREHAND'], STATES['BACKHAND']]

SPINS = {}
SPINS['TOPSPIN'] = 0
SPINS['BACKSPIN'] = 1

# set pwm duty cycle for different spins
BACKSPIN_PWM = {'top':30, 'bottom':50}
TOPSPIN_PWM = {'top':50, 'bottom':30}

# set sweep direction
COUNTERCLOCKWISE = {'GEAR_DIR_A':GPIO.LOW, 'GEAR_DIR_B':GPIO.HIGH}
CLOCKWISE = {'GEAR_DIR_A':GPIO.HIGH, 'GEAR_DIR_B':GPIO.LOW}

PWM_PINS = ['SERVO', 'MB_PWM', 'MT_PWM', 'GEAR_PWM']
SERVO_PWM_FREQ = 50  # 50Hz, or 20ms pulse cycle from spec sheet
PWM_FREQ = 1000 # for non servo pwm


# encoder
TICKS_PER_REV = 48
RADS_PER_TICK = 1.0 / TICKS_PER_REV * 6.28
LEN_MOVING_AVERAGE = 3

# motor constant (guesstimating)
k_motor = 0.01  # Volts * s / rad
R_motor = 22    # Ohms

# robot params (derived on paper)
# rotation moment of inertia (approximate as rectangular prism)
Izz = (1.0 / 12) * 3 * (0.09**2 + 0.21**2)  # kg * m^2
d = 0.09    # m
D = 0.03    # m
GEAR_RATIO = 100
THETADES_TO_THETAMOTOR = GEAR_RATIO * d / D
THETADES = {'FOREHAND':0.5, 'BACKHAND':-0.5}
MAX_VOLTAGE = 8     # after accounting for loss in L298N driver

# feedback control, found via root locus and guess/check
kp = 110
kd = 0.4
SETTLING_TIME = 0.5     

