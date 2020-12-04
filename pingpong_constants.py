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

SPINS = {}
SPINS['TOPSPIN'] = 0
SPINS['BACKSPIN'] = 1

# set pwm duty cycle for different spins
BACKSPIN_PWM = {"top":30, "bottom":50}
TOPSPIN_PWM = {"top":50, "bottom":30}

PWM_PINS = ['SERVO', 'MB_PWM', 'MT_PWM', 'GEAR_PWM']
SERVO_PWM_FREQ = 50  # 50Hz, or 20ms pulse cycle from spec sheet
PWM_FREQ = 1000 # for non servo pwm

