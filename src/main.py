# Library imports
from vex import *
import math

# Brain should be defined by default
brain = Brain()

# Robot configuration code
controller = Controller(PRIMARY)

left_motor1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True) # left top
left_motor2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
left_motor3 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
right_motor1 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False) # right top
right_motor2 = Motor(Ports.PORT5, GearSetting.RATIO_18_1, True)
right_motor3 = Motor(Ports.PORT6, GearSetting.RATIO_18_1, True)

intake_motor = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)

backpack_lift_thingy = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)

WHEEL_DIAMETER = 4

getLeftEncoderValue = lambda : left_motor3.position(DEGREES) / 360 * WHEEL_DIAMETER * math.pi # because je suis too lazy to type left_motor3 15 times
getRightEncoderValue = lambda : right_motor3.position(DEGREES) / 360 * WHEEL_DIAMETER * math.pi

MOTOR_DISTANCE_FROM_CENTER = 10

TURNING_DISTANCE = 2 * MOTOR_DISTANCE_FROM_CENTER * math.pi # in inches

position_x, position_y, theta = 0, 0, 0

pneumatic_calibration_array = lambda : [[[[[[0] * 31] * 31] * 31] * 31] * 31] * 31

# PID CONSTANT TERMS
Kp, Ki, Kd = 1.0, 0.1, 0

# VISION
# visionSensor = Vision(Ports.PORT10)
# # Example signature with Hue (Hue Min, Hue Max), Saturation (Sat Min, Sat Max), and Brightness (Bright Min, Bright Max)
# # Define a color signature for red
# # Parameters: (id, uMin, uMax, uMean, vMin, vMax, vMean, range, type)
# SIGNATURE_RED = Signature( # will do same for blue
#     1,        # Signature ID
#     9000,     # uMin (Hue)
#     12000,    # uMax (Hue)
#     10500,    # uMean (Hue mean)
#     5000,     # vMin (Saturation)
#     7000,     # vMax (Saturation)
#     6000,     # vMean (Saturation mean)
#     3.0,      # sigrange (Range of values to consider as part of the signature / tolerance)
#     0         # sigtype (Type: 0 = Normal, 1 = Advanced)
# )


# Functions
def spin_motors():
    left_motor1.spin(FORWARD)
    left_motor2.spin(FORWARD)
    left_motor3.spin(FORWARD)
    right_motor1.spin(FORWARD)
    right_motor2.spin(FORWARD)
    right_motor3.spin(FORWARD)

def set_motor_velocities(left_speed: float, right_speed: float):
    global position_x, position_y, theta
    # INSERT POSITION TRACKING CODE
    
    left_motor1.set_velocity(left_speed, PERCENT)
    left_motor2.set_velocity(left_speed, PERCENT)
    left_motor3.set_velocity(left_speed, PERCENT)
    right_motor1.set_velocity(right_speed, PERCENT)
    right_motor2.set_velocity(right_speed, PERCENT)
    right_motor3.set_velocity(right_speed, PERCENT)
    spin_motors()
    
def run_intake(forward: bool, reverse: bool):
    if (forward): # one direction
        intake_motor.set_velocity(100, PERCENT)
    elif (reverse): # the other
        intake_motor.set_velocity(-100, PERCENT)
    else:
        intake_motor.set_velocity(0, PERCENT)
    intake_motor.spin(FORWARD)

def movePI(distance: float):  # forward is positive, distance in inches
    global integral  # Use the global integral variable
    global position_x, position_y
    startingPosition = getLeftEncoderValue()
    targetPosition = startingPosition + distance
    integral = 0  # Reset the integral term at the start of the movement

    while (abs(targetPosition - getLeftEncoderValue()) > 0.1):
        currentPosition = getLeftEncoderValue()
        error = targetPosition - currentPosition

        # Proportional term
        kP = Kp * error

        # Integral term
        integral += error  # Accumulate error
        kI = Ki * integral
        
        # Derivative Term
        # NOT IMPLEMENTED

        # Calculate motor speeds
        left_speed = kP + kI
        right_speed = kP + kI

        set_motor_velocities(left_speed, right_speed)
        wait(5, MSEC)

    # Stop motors after reaching the target position
    set_motor_velocities(0, 0)

def rotate(degrees: float): # right is positive
    degrees %= 360 # prevent the robot from rotating 15000 times
    if degrees <= 180: # if faster to turn right
        left_target = getLeftEncoderValue() + degrees / 360 * TURNING_DISTANCE
        right_target = getRightEncoderValue() - degrees / 360 * TURNING_DISTANCE
    else: # if faster to turn left
        left_target = getLeftEncoderValue() - degrees / 360 * TURNING_DISTANCE
        right_target = getRightEncoderValue() + degrees / 360 * TURNING_DISTANCE
        
    # INSERT PID CONTROL CODE HERE
    while (abs(left_target - getLeftEncoderValue()) > 0.01 and abs(right_target - getRightEncoderValue()) > 0.01): # more precision required
        error = ((left_target - getLeftEncoderValue()) + (right_target - getRightEncoderValue())) / 2
        rotate_speed = 0
        set_motor_velocities(rotate_speed, -rotate_speed)
    set_motor_velocities(0, 0)
    
def goTo(x: float, y: float):
    required_theta = math.degrees(math.atan((y - position_y) / (x - position_x)))
    degrees_to_turn = theta - required_theta if theta - required_theta >= 0 else 360 + theta - required_theta
    rotate(degrees_to_turn)
    movePI(math.sqrt((y - position_y) ** 2 + (x - position_x) ** 2))

# put all autonomous code here:
def autonomous():
    movePI(10)

# MAIN LOOP to set motors to controller axis positions
while True:
    set_motor_velocities(controller.axis3.position() - controller.axis1.position(), controller.axis3.position() + controller.axis1.position())
    
    run_intake(controller.buttonR2.pressing(), controller.buttonL2.pressing())
    
    # objects = visionSensor.take_snapshot(SIGNATURE_RED) # SHOULD CHECK, mayhaps steal from the internet
    # object = visionSensor.largest_object()
    # if object.height > 50 and object.id == 1: # 1 - Red ring, 2 - Blue (0 - mobile goal)
    #     pass # do something if object is large enough
    wait(5, MSEC)