__all__  = ['WHEEL_RADIUS', 'WHEEL_GAP', 'PWM_UP_LIMIT', 'PWM_DOWN_LIMIT', 'LINEAR_VELOCITY_MAX_LIMIT', 'LINEAR_VELOCITY_MIN_LIMIT'
'ANGULAR_VELOCITY_MAX_LIMIT', 'ANGULAR_VELOCITY_MIN_LIMIT', 'PID_SCAN_FREQUENCY', 'PID_ERROR_STACK']


WHEEL_RADIUS = 0.03
WHEEL_GAP = 0.19

PWM_UP_LIMIT = 100
PWM_DOWN_LIMIT = 0

LINEAR_VELOCITY_MAX_LIMIT = 0.8
LINEAR_VELOCITY_MIN_LIMIT = -0.8
ANGULAR_VELOCITY_MAX_LIMIT = 1.0
ANGULAR_VELOCITY_MIN_LIMIT = -1.0

PID_SCAN_FREQUENCY = 0.4
PID_ERROR_STACK = 10