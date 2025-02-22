#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cstdlib>

std::string current_auton = "red+"; // skills, red left, red right, blue left, blue right
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
ASSET(AltSkills_Txt);

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
pros::Motor climb(9, pros::MotorGearset::red);

pros::adi::DigitalOut clamp('D');
bool clamp_state = false;

// motor groups
pros::MotorGroup rightMotors({-20, -19, 18}, pros::MotorGearset::green);
pros::MotorGroup leftMotors({11, 13, -12}, pros::MotorGearset::green);

// dimensions: width 12.75, length 15.5 <-- Remeasure this from the middle of the middle wheels because that maye be a source of ERROR in the auton
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.75, lemlib::Omniwheel::NEW_325, 200.0 * 5 / 3, 8);
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, nullptr);

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
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
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
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
    chassis.calibrate();
    // current_auton = 1;
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
        chassis.moveToPoint(0, 24, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
    } else if (current_auton == "skills") { // upload files onto path.jerryio.com for visualization
        toggle_clamp();
        chassis.setPose(-60.574, -0.13, 90);
        chassis.follow(Skills1_txt, 2, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        pros::delay(1000);
        chassis.turnToHeading(300, 2000);
        chassis.follow(Skills2_txt, 1, 110000, false);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(false, false);
        chassis.follow(Skills3_txt, 1, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.follow(Skills4_txt, 1, 110000, false);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(false, false);
        chassis.follow(Skills5_txt, 1, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.follow(Skills6_txt, 1, 110000, false);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(false, false);
        chassis.follow(Skills7_txt, 1, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.follow(Skills8_txt, 1, 110000, false);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(false, false);
        chassis.follow(Skills9_txt, 1, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.follow(Skills10_txt, 1, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(false, false);
    } else if (current_auton == "red+") {
        chassis.setPose(-58.467, -56.047, 90);
        chassis.follow(redPos1_txt, 1, 110000);
        chassis.waitUntilDone();
        toggle_clamp();
        chassis.turnToHeading(180, 1000);
        chassis.waitUntilDone();
        toggle_clamp();
        chassis.turnToHeading(0, 1000);
        chassis.follow(redPos2_txt, 1, 110000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.follow(redPos3_txt, 1, 110000, false);
    } else if (current_auton == "red-") {
        chassis.setPose(-58.659, 41.807, 120);
        chassis.follow(redNeg1_txt, 1, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.turnToHeading(240, 1000);
        chassis.follow(redNeg2_txt, 1, 10000, false);
        chassis.waitUntilDone();
        run_intake(false, false);
        chassis.follow(redNeg3_txt, 1, 10000);
        chassis.waitUntilDone();
        run_intake(true, false);
    } else if (current_auton == "blue+") {
        chassis.setPose(58.467, -56.047, 90);
        chassis.follow(bluePos1_txt, 1, 110000);
        chassis.waitUntilDone();
        toggle_clamp();
        chassis.turnToHeading(180, 1000);
        chassis.waitUntilDone();
        toggle_clamp();
        chassis.turnToHeading(0, 1000);
        chassis.follow(bluePos2_txt, 1, 110000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.follow(bluePos3_txt, 1, 110000, false);
    } else if (current_auton == "blue-") {
        chassis.setPose(58.659, 41.807, 120);
        chassis.follow(blueNeg1_txt, 1, 10000);
        chassis.waitUntilDone();
        toggle_clamp();
        run_intake(true, false);
        chassis.turnToHeading(120, 1000);
        chassis.follow(blueNeg2_txt, 1, 10000, false);
        chassis.waitUntilDone();
        run_intake(false, false);
        chassis.follow(blueNeg3_txt, 1, 10000);
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

		climb.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        if (abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 0.05) {
            climb.brake();
        }

        // delay to save resources
        pros::delay(25);
    }
}