# Library imports
from vex import *
import math
# Documentation - https://api.vex.com/v5/home/python/index.html

# CONSTANTS - ALL DISTANCES ARE IN INCHES - SUBJECT TO CHANGE
WHEEL_CIRCUMFERENCE = math.pi * 3.25
TURNING_DIAMETER = 10
TURNING_DISTANCE = TURNING_DIAMETER * math.pi # in inches
MAX_MOTORS_DEGREES_PER_5_MS = 200 / 60 / 1000 * 5 / 360 # the maximum number of degrees a v5 motor with 200 rpm can rotate every 5 milliseconds

# DRIVER PREFERENCES
FORWARD_MULTIPLIER = 0.6
TURNING_MULTIPLIER = 0.45

# Brain should be defined by default
brain = Brain()

# Robot configuration code
controller = Controller(PRIMARY)

left_motor1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False) # left top
left_motor2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)
left_motor3 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
right_motor1 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True) # right top
right_motor2 = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)
right_motor3 = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)

intake_motor = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)

climb_motor = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)

clamp = DigitalOut(brain.three_wire_port.c)

# backpack_lift_thingy = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
left_motor3.reset_position()
right_motor3.reset_position()

getLeftEncoderValue = lambda : left_motor3.position(DEGREES) / 360 * WHEEL_CIRCUMFERENCE # because I'm lazy
getRightEncoderValue = lambda : right_motor3.position(DEGREES) / 360 * WHEEL_CIRCUMFERENCE

# POSITION TRACKING
position_x, position_y, theta = 0, 0, 0

# PID CONSTANT TERMS
Kp, Ki, Kd = 0.5, 0, 0

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
    
def brake_motors():
    left_motor1.stop(BRAKE)
    left_motor2.stop(BRAKE)
    left_motor3.stop(BRAKE)
    right_motor1.stop(BRAKE)
    right_motor2.stop(BRAKE)
    right_motor3.stop(BRAKE)

def set_motor_velocities(left_speed: float, right_speed: float):
    global position_x, position_y, theta
    
    # DEADZONES for the motors
    if (-5 < left_speed < 5):
        left_speed = 0
    elif (left_speed > 100):
        left_speed = 100
    elif (left_speed < -100):
        left_speed = -100
    
    if (-5 < right_speed < 5):
        right_speed = 0
    elif (right_speed > 100):
        right_speed = 100
    elif (right_speed < -100):
        right_speed = -100
    
    # POSITION TRACKING CODE
    position_x += math.cos(theta * math.pi / 180) * (left_speed + right_speed) / 200 * MAX_MOTORS_DEGREES_PER_5_MS / 360 * WHEEL_CIRCUMFERENCE
    position_y += math.sin(theta * math.pi / 180) * (left_speed + right_speed) / 200 * MAX_MOTORS_DEGREES_PER_5_MS / 360 * WHEEL_CIRCUMFERENCE
    
    # THETA TRACKING
    if (left_speed == -right_speed): # position tracking won't be used during out of autonomous so not necessary to account for both moving and spinning
        theta += ((left_speed / 100 * MAX_MOTORS_DEGREES_PER_5_MS) * WHEEL_CIRCUMFERENCE) / TURNING_DISTANCE * 360
        theta %= 360
    
    left_motor1.set_velocity(left_speed, VelocityUnits.PERCENT)
    left_motor2.set_velocity(left_speed, VelocityUnits.PERCENT)
    left_motor3.set_velocity(left_speed, VelocityUnits.PERCENT)
    right_motor1.set_velocity(right_speed, VelocityUnits.PERCENT)
    right_motor2.set_velocity(right_speed, VelocityUnits.PERCENT)
    right_motor3.set_velocity(right_speed, VelocityUnits.PERCENT)
    
    if left_speed == 0 and right_speed == 0:
        brake_motors()
    else:
        spin_motors()
    
def run_intake(forward: bool, reverse: bool):
    if (forward): # one direction
        intake_motor.set_velocity(200, VelocityUnits.RPM)
    elif (reverse): # the other
        intake_motor.set_velocity(-200, VelocityUnits.RPM)
    else:
        intake_motor.set_velocity(0, VelocityUnits.RPM)
    intake_motor.spin(FORWARD)
    
def run_climb(speed: float):
    if abs(speed) <= 5:
        speed = 0
        climb_motor.stop(BRAKE)
        return
    elif speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    climb_motor.set_velocity(speed, VelocityUnits.PERCENT)
    climb_motor.spin(FORWARD)

def toggle_clamp(): # extends clamp pistons if not extended and vice versa
    clamp.set(not clamp.value())

def move(distance: float):  # forward is positive, distance in inches
    global position_x, position_y
    distance /= 1.4
    
    startingPosition = getLeftEncoderValue()
    targetPosition = startingPosition + distance
    integral = 0  # Reset the integral term at the start of the movement
    
    previous_error = targetPosition - startingPosition

    while (abs(targetPosition - getLeftEncoderValue()) > 0.1):
        currentPosition = getLeftEncoderValue()
        error = targetPosition - currentPosition

        # Proportional term
        kP = Kp * error

        # Integral term
        integral += error  # Accumulate error
        kI = Ki * integral
        
        # Derivative Term
        kD = Kd * (error - previous_error) / 0.005 # change in error divided by change in time

        # Calculate motor speeds
        left_speed = (kP + kI + kD) * 100 # since its in percent
        right_speed = (kP + kI + kD) * 100

        set_motor_velocities(left_speed, right_speed)
        
        previous_error = error
        wait(5, MSEC)

    # Stop motors after reaching the target position
    brake_motors()

def rotate(degrees: float): # right is positive
    degrees %= 360 # prevent the robot from rotating 15000 times
    
    if degrees <= 180: # if faster to turn right
        left_target = getLeftEncoderValue() + degrees / 360 * TURNING_DISTANCE
        right_target = getRightEncoderValue() - degrees / 360 * TURNING_DISTANCE
    else: # if faster to turn left
        left_target = getLeftEncoderValue() - degrees / 360 * TURNING_DISTANCE
        right_target = getRightEncoderValue() + degrees / 360 * TURNING_DISTANCE
        
    left_integral = 0
    right_integral = 0
    
    while (abs(left_target - getLeftEncoderValue()) > 0.01 and abs(right_target - getRightEncoderValue()) > 0.01): # more precision required
        left_error = left_target - getLeftEncoderValue()
        right_error = right_target - getRightEncoderValue()
        
        left_kP = Kp * left_error
        right_kP = Kp * right_error
        
        left_integral += left_error
        right_integral += right_error
        
        left_kI = Ki * left_integral
        right_kI = Ki * right_integral
        
        set_motor_velocities(left_kP + left_kI, right_kP + right_kI)
        wait(5, MSEC)
    set_motor_velocities(0, 0)
    
def goTo(x: float, y: float):
    required_theta = math.degrees(math.atan((y - position_y) / (x - position_x)))
    degrees_to_turn = theta - required_theta if theta - required_theta >= 0 else 360 + theta - required_theta
    rotate(degrees_to_turn)
    move(math.sqrt((y - position_y) ** 2 + (x - position_x) ** 2))

def pre_auton():
    left_motor3.reset_position()
    right_motor3.reset_position()

# put all autonomous code here:
def autonomous():
    # move(23)
    clamp.set(True)

# driver control period
def drive_task():
    controller.buttonR1.pressed(toggle_clamp)
    
    while True:
        set_motor_velocities(FORWARD_MULTIPLIER * -1 * controller.axis3.position() - TURNING_MULTIPLIER * controller.axis4.position(), FORWARD_MULTIPLIER * -1 * controller.axis3.position() + TURNING_MULTIPLIER *  controller.axis4.position())
    
        run_intake(controller.buttonL2.pressing(), controller.buttonR2.pressing())
        
        run_climb(controller.axis2.position())
    
        # objects = visionSensor.take_snapshot(SIGNATURE_RED) # SHOULD CHECK, mayhaps steal from the internet
        # object = visionSensor.largest_object()
        # if object.height > 50 and object.id == 1: # 1 - Red ring, 2 - Blue (0 - mobile goal)
        #     pass # do something if object is large enough
        wait(5, MSEC)

competition = Competition(drive_task, autonomous)

pre_auton()

# autonomous()
# drive_task()