#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "pros/misc.h"

std::string auton = "skills"; // skills, red left, red right, blue left, blue right
// path loading
ASSET(blueLeft1_txt);
ASSET(blueLeft2_txt);
ASSET(blueLeft3_txt);
ASSET(blueRight1_txt);
ASSET(blueRight2_txt);
ASSET(blueRight3_txt);
ASSET(redRight1_txt);
ASSET(redRight2_txt);
ASSET(redRight3_txt);
ASSET(redLeft1_txt);
ASSET(redLeft2_txt);
ASSET(redLeft3_txt);
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

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor intake(-10, pros::MotorGearset::green);
// pros::Motor climb(9, pros::MotorGearset::green);

pros::adi::DigitalOut clamp('D');
bool clamp_state = false;

// motor groups
pros::MotorGroup leftMotors({20, -19, -18}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({-11, 13, 12}, pros::MotorGearset::green);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10, lemlib::Omniwheel::NEW_325, 200, 0);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, nullptr);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

void toggle_clamp() {
	clamp.set_value(!clamp_state);
	clamp_state = !clamp_state;
}

void run_intake(bool out, bool in) {
    if (out) {
		intake.move(127);
	} else if (in) {
		intake.move(-127);
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
    if (auton == "skills") { // upload files onto path.jerryio.com for visualization
        chassis.setPose(0, 0, 0);
        chassis.follow(Skills1_txt, 15, 5000);
        toggle_clamp();
        run_intake(false, true);
        chassis.follow(Skills2_txt, 15, 15000, false);
        toggle_clamp();
        run_intake(false, false);
        chassis.follow(Skills3_txt, 15, 5000);
        toggle_clamp();
        run_intake(false, true);
        chassis.follow(Skills4_txt, 15, 15000, false);
        toggle_clamp();
        run_intake(false, false);
        chassis.follow(Skills5_txt, 15, 5000);
        toggle_clamp();
        run_intake(false, true);
        chassis.follow(Skills6_txt, 15, 15000, false);
        toggle_clamp();
        run_intake(false, false);
        chassis.follow(Skills7_txt, 15, 5000);
        toggle_clamp();
        run_intake(false, true);
        chassis.follow(Skills8_txt, 15, 15000, false);
        toggle_clamp();
        run_intake(false, false);
        chassis.follow(Skills9_txt, 15, 5000);
        toggle_clamp();
        run_intake(false, true);
        chassis.follow(Skills10_txt, 15, 5000);
        toggle_clamp();
        run_intake(false, false);
    } else if (auton == "red left") {
        chassis.setPose(-58.467, 23.615, 86.917);
        chassis.follow(redLeft1_txt, 15, 5000);
        toggle_clamp();
        run_intake(false, true);
        chassis.follow(redLeft2_txt, 15, 5000, false);
        run_intake(false, false);
        chassis.follow(redLeft3_txt, 15, 5000);
    } else if (auton == "red right") {
        chassis.setPose(-63.515, -25.216, 89.147);
        chassis.follow(redRight1_txt, 15, 5000);
        toggle_clamp();
        run_intake(false, true);
        chassis.follow(redRight2_txt, 15, 5000, false);
        run_intake(false, false);
        chassis.follow(redRight3_txt, 15, 5000);
    } else if (auton == "blue left") {
        chassis.setPose(31.152, -28.28, 95.058);
        chassis.follow(blueLeft1_txt, 15, 5000);
        toggle_clamp();
        run_intake(false, true);
        chassis.follow(blueLeft2_txt, 15, 5000, false);
        run_intake(false, false);
        chassis.follow(blueLeft3_txt, 15, 5000);
    } else if (auton == "blue right") {
        chassis.setPose(58.467, 23.615, 86.915);
        chassis.follow(blueRight1_txt, 15, 5000);
        toggle_clamp();
        run_intake(false, true);
        chassis.follow(blueRight2_txt, 15, 5000, false);
        run_intake(false, false);
        chassis.follow(blueRight3_txt, 15, 5000);
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
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(0.6 * leftY, 0.6 * rightY);

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
			toggle_clamp();
		}

		run_intake(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2), controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2));

		// climb.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

        // delay to save resources
        pros::delay(25);
    }
}