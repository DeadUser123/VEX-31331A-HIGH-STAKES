#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include <cstdlib>

std::string current_auton = "skills"; // skills, red left, red right, blue left, blue right
// path loading
// ASSET(blueLeft1_txt);
// ASSET(blueLeft2_txt);
// ASSET(blueLeft3_txt);
// ASSET(blueRight1_txt);
// ASSET(blueRight2_txt);
// ASSET(blueRight3_txt);
// ASSET(redRight1_txt);
// ASSET(redRight2_txt);
// ASSET(redRight3_txt);
// ASSET(redLeft1_txt);
// ASSET(redLeft2_txt);
// ASSET(redLeft3_txt);
ASSET(Skills1_txt);
ASSET(Skills2_txt);
ASSET(Skills3_txt);
ASSET(Skills4_txt);
ASSET(Skills5_txt);
ASSET(Skills6_txt);
ASSET(Skills7_txt);
ASSET(Skills8_txt);
ASSET(Skills9_txt);
ASSET(Skills10_txt);
ASSET(AltSkills_txt);

ASSET(redPos1_txt);
ASSET(redPos2_txt);
ASSET(redPos3_txt);

ASSET(redNeg1_txt);
ASSET(redNeg2_txt);
ASSET(redNeg3_txt);

ASSET(bluePos1_txt);
ASSET(bluePos2_txt);
ASSET(bluePos3_txt);

ASSET(blueNeg1_txt);
ASSET(blueNeg2_txt);
ASSET(blueNeg3_txt);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor intake(-10, pros::MotorGearset::blue);
pros::Motor climb(9, pros::MotorGearset::green);

pros::adi::DigitalOut clamp('D');
bool clamp_state = false;

// motor groups
pros::MotorGroup rightMotors({20, 19, -18}, pros::MotorGearset::green);
pros::MotorGroup leftMotors({-11, -13, 12}, pros::MotorGearset::green);

pros::IMU imu(1);
pros::Rotation horizontal_encoder(2);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, 0.1);

pros::Rotation climb_encoder(3);
double climbProfileConveyorPos = -0.04 * 360 * 100;

int lookahead = 6;

// pros::Vision vision_sensor(0);

// dimensions: width 12.75, length 15.5 <-- Remeasure this from the middle of the middle wheels because that maye be a source of ERROR in the auton
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.75, lemlib::Omniwheel::NEW_325, 200.0 * 5 / 3, 8);
lemlib::OdomSensors sensors(nullptr, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0.4 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              0.1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

void toggle_clamp() {
	clamp.set_value(!clamp_state);
	clamp_state = !clamp_state;
}

void goToClimb() {
    double climb_kP = 0.1, climb_kI = 0, climb_kD = 0;
    double climb_integral = 0, climb_prev_error = 0;
    int target_position = climbProfileConveyorPos;
    while (true) {
        // Get current position from encoder
        int current_position = climb_encoder.get_position();
        
        // Calculate error
        double climb_error = target_position - current_position;
        
        // Integral term (accumulate error over time)
        climb_integral += climb_error;
        
        // Derivative term (rate of change of error)
        double climb_derivative = climb_error - climb_prev_error;
        climb_prev_error = climb_error;

        // PID Output
        double climb_power = (climb_kP * climb_error) + (climb_kI * climb_integral) + (climb_kD * climb_derivative);
        
        // Clamp power between -127 and 127
        climb_power = std::clamp(climb_power, -127.0, 127.0);
        
        // Apply power to motor
        climb.move(climb_power);

        // Stop the loop when close to target
        if (std::abs(climb_error) < 100) {
            climb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            climb.brake();
            break;
        }

        pros::delay(20); // Delay for stability
    }
}

void run_intake(bool in, bool out) {
    if (in) {
		intake.move(0.9 * 127);
	} else if (out) {
		intake.move(0.9 * -127);
	} else {
		intake.move(0);
	}
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Intialized");
	pros::lcd::register_btn1_cb(on_center_button);
    climb_encoder.reset_position();
    chassis.calibrate();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    if (current_auton == "test") {
        chassis.setPose(0, 0, 0);
        toggle_clamp();
        // chassis.moveToPoint(0, 24, 10000);
        chassis.turnToHeading(90, 5000);
        chassis.waitUntilDone();
        toggle_clamp();
        // run_intake(true, false);
    } else if (current_auton == "skills") { // upload files onto path.jerryio.com for visualization
        toggle_clamp();
        chassis.setPose(-60.574, -0.13, 90);
        chassis.follow(Skills1_txt, lookahead, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        pros::delay(1000);
        chassis.turnToHeading(300, 2000);
        chassis.follow(Skills2_txt, lookahead, 110000, false);
        chassis.waitUntilDone();
        chassis.turnToHeading(225, 1000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(false, false);
        pros::delay(500);
        chassis.turnToHeading(0, 500);
        chassis.follow(Skills3_txt, lookahead, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.follow(Skills4_txt, lookahead, 110000, false);
        chassis.waitUntilDone();
        chassis.turnToHeading(315, 1000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(false, false);
        pros::delay(500);
        chassis.turnToHeading(135, 1000);
        chassis.waitUntilDone();
        chassis.follow(Skills5_txt, lookahead, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        pros::delay(500);
        chassis.follow(Skills6_txt, lookahead, 110000, false);
        chassis.waitUntilDone();
        chassis.turnToHeading(45, 1000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(false, false);
        pros::delay(500);
        chassis.turnToHeading(180, 1000);
        chassis.waitUntilDone();
        chassis.follow(Skills7_txt, lookahead, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.follow(Skills8_txt, lookahead, 110000, false);
        chassis.waitUntilDone();
        chassis.turnToHeading(125, 1000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(false, false);
        pros::delay(500);
        chassis.turnToHeading(315, 1000);
        chassis.waitUntilDone();
        chassis.follow(Skills9_txt, lookahead, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.follow(Skills10_txt, lookahead, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(false, false);
    } else if (current_auton == "red+") {
        chassis.setPose(-59.042, -35.557, 90);
        chassis.follow(redPos1_txt, lookahead, 110000);
        chassis.waitUntilDone();
        toggle_clamp();
        pros::delay(500);
        chassis.turnToHeading(180, 1000);
        chassis.waitUntilDone();
        toggle_clamp();
        pros::delay(500);
        chassis.turnToHeading(0, 1000);
        chassis.follow(redPos2_txt, lookahead, 110000);
        chassis.waitUntilDone();
        toggle_clamp();
        pros::delay(500);
        run_intake(true, false);
        chassis.follow(redPos3_txt, lookahead, 110000, false);
    } else if (current_auton == "red-") {
        chassis.setPose(-58.659, 41.807, 120);
        chassis.follow(redNeg1_txt, lookahead, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.turnToHeading(240, 1000);
        chassis.follow(redNeg2_txt, lookahead, 10000, false);
        chassis.waitUntilDone();
        run_intake(false, false);
        chassis.follow(redNeg3_txt, lookahead, 10000);
        chassis.waitUntilDone();
        run_intake(true, false);
    } else if (current_auton == "blue+") {
        chassis.setPose(58.467, -56.047, 90);
        chassis.follow(bluePos1_txt, lookahead, 110000);
        chassis.waitUntilDone();
        toggle_clamp();
        chassis.turnToHeading(180, 1000);
        chassis.waitUntilDone();
        toggle_clamp();
        chassis.turnToHeading(0, 1000);
        chassis.follow(bluePos2_txt, lookahead, 110000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.follow(bluePos3_txt, lookahead, 110000, false);
    } else if (current_auton == "blue-") {
        chassis.setPose(58.659, 41.807, 120);
        chassis.follow(blueNeg1_txt, lookahead, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.turnToHeading(120, 1000);
        chassis.follow(blueNeg2_txt, lookahead, 10000, false);
        chassis.waitUntilDone();
        run_intake(false, false);
        chassis.follow(blueNeg3_txt, lookahead, 10000);
        chassis.waitUntilDone();
        run_intake(true, false);
    }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
pros::Task* climbThread = nullptr;

void opcontrol() {
	
	while (true) {
        // get left y and right y positions
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        // move the robot
        chassis.arcade(-0.7 * leftY, 0.5 * leftX);

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
			toggle_clamp();
		}
        
		run_intake(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2), controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2));

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            delete climbThread;
            climbThread = nullptr;
            climbThread = new pros::Task(goToClimb);
        }

        if ((climbThread != nullptr && climbThread->get_state() == pros::E_TASK_STATE_DELETED) || controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            delete climbThread; 
            climbThread = nullptr;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            climb_encoder.reset_position();
        }

		climb.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        if (abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) < 0.05) {
            climb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            climb.brake();
        } else {
            climb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }

        // delay to save resources
        pros::delay(25);
    }
}