# Library imports
from vex import *

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

# VISION
# visionSensor = Vision(Ports.PORT1)
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
def set_motor_velocities():
    left_speed = controller.axis3.position() - controller.axis1.position()
    right_speed = controller.axis3.position() + controller.axis1.position()
    
    left_motor1.set_velocity(left_speed, PERCENT)
    left_motor2.set_velocity(left_speed, PERCENT)
    left_motor3.set_velocity(left_speed, PERCENT)
    right_motor1.set_velocity(right_speed, PERCENT)
    right_motor2.set_velocity(right_speed, PERCENT)
    right_motor3.set_velocity(right_speed, PERCENT)

def spin_motors():
    left_motor1.spin(FORWARD)
    left_motor2.spin(FORWARD)
    left_motor3.spin(FORWARD)
    right_motor1.spin(FORWARD)
    right_motor2.spin(FORWARD)
    right_motor3.spin(FORWARD)

# MAIN LOOP to set motors to controller axis positions
while True:
    set_motor_velocities()
    spin_motors()
    
    if (controller.buttonA.pressing()): # one direction
        intake_motor.set_velocity(100, PERCENT)
    elif (controller.buttonB.pressing()): # the other
        intake_motor.set_velocity(-100, PERCENT)
    else:
        intake_motor.set_velocity(0, PERCENT)
    intake_motor.spin(FORWARD)
        
    # objects = visionSensor.take_snapshot(SIGNATURE_RED) # SHOULD CHECK
    # object = visionSensor.largest_object()
    # if object.height > 50 and object.id == 1: # 1 - Red ring, 2 - Blue (0 - mobile goal)
    #     pass # do something if object is large enough
    wait(5, MSEC)