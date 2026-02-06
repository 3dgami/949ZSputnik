#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include <cstdio>
//#include "lemlib-tarball/api.hpp"

pros::Controller master{CONTROLLER_MASTER};	
pros::MotorGroup driveL_train({-13, -11, 12}, pros::v5::MotorGears::blue);//UPDATE WITH MOTOR WIRING CHANGING
pros::MotorGroup driveR_train({18, 20, -19}, pros::v5::MotorGears::blue);

pros::Motor intake1(10, pros::v5::MotorGears::blue);
//pros::Motor backstack(8, pros::v5::MotorGears::green);
pros::Motor intake2(-9, pros::v5::MotorGears::green);

pros::IMU imu(17);

pros::Rotation Horizontal_rotation(16);
lemlib::TrackingWheel horizontal_tracking(&Horizontal_rotation, lemlib::Omniwheel::NEW_275, -4.25);//configure offset


pros::Rotation Vertical_rotation(4);
lemlib::TrackingWheel vertical_tracking(&Vertical_rotation, lemlib::Omniwheel::NEW_275, -0.4);//configure offset

bool matchloaderState = false;
int IntakeState = 0;
bool descorerState = false;

pros::adi::DigitalOut matchloader('A');
pros::adi::DigitalOut descorer('B');





// drivetrain settings //UPDATE
lemlib::Drivetrain drivetrain(&driveL_train, // left motor group
                              &driveR_train, // right motor group
                              11, // 11 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(15, // proportional gain (kP)
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
lemlib::ControllerSettings angular_controller(1.9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              500, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              1000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking,// horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     40, // minimum output where drivetrain will move out of 127
                                     1.024 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  40, // minimum output where drivetrain will move out of 127
                                  1.024 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


//AUTON//
ASSET(Newtest_txt);
ASSET(Newtest1_txt);
ASSET(Newtest2_txt);
//LEFT SIDE//

//RIGHT SIDE//

//LEFT SIDE ELIMS//

//RIGHT SIDE ELIMS//

//SKILLS//
ASSET(Skills1_txt);
ASSET(Skills2_txt);
ASSET(Skills3_txt);
ASSET(Skills4_txt);
ASSET(Skills5_txt);
ASSET(Skills6_txt);
ASSET(Skills7_txt);
ASSET(Skills8_txt);
ASSET(Skills9_txt);

void RighSide() {}

void LeftSide() {}

void RighSideElims() {}

void LeftSideElims() {}

void Skills() {

	chassis.setPose(47.936, -13.232, 180);

	chassis.follow(Skills1_txt, 15, 5000, true);

	chassis.turnToHeading(90, 1000);

	chassis.follow(Skills2_txt, 15, 5000, true);

	chassis.waitUntilDone();
	pros::delay(1000);

	chassis.follow(Skills3_txt, 15, 3000, false);

	chassis.waitUntilDone();
	pros::delay(1000);

	chassis.follow(Skills4_txt, 15, 6000, true);

	chassis.waitUntilDone();

	chassis.turnToHeading(225, 1000);

	chassis.waitUntilDone();
	pros::delay(1000);

	chassis.follow(Skills5_txt, 15, 10000, true);

	chassis.turnToHeading(270, 1000);
	chassis.waitUntilDone();
	pros::delay(1000);
	chassis.setPose(29,-59, 270);

	chassis.follow(Skills6_txt, 15, 10000, true);

	chassis.turnToHeading(315, 1000);


	chassis.follow(Skills7_txt, 15, 10000, true);

	chassis.turnToHeading(270, 1000);

	chassis.follow(Skills8_txt, 15, 5000, true);

	chassis.waitUntilDone();
	pros::delay(500);

	chassis.follow(Skills9_txt, 15, 5000, false);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize(); // initialize brain screen
	//master.set_text(0,5, "Phillipines 4:13");
    chassis.calibrate(); // calibrate sensors

	chassis.setPose(0, 0, 0);
	
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
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
void competition_initialize() {

}

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

	//chassis.calibrate();
	
	/*pros::delay(500);
	
	chassis.setPose(-44.823, 13.588, 90);
	//matchloader.set_value(HIGH);
	chassis.follow(Newtest_txt, 15, 5000, true);
	pros::delay(500);
	matchloader.set_value(HIGH);
	chassis.turnToHeading(125, 1000);
	chassis.follow(Newtest1_txt, 15, 5000, true);
	pros::delay(500);
	
	pros::delay(3000);
	chassis.follow(Newtest2_txt, 15, 5000, false);
	chassis.waitUntilDone();
	chassis.setPose(-45.498, 47.277, 125);
	chassis.waitUntilDone();
	chassis.turnToHeading(235, 1000);
	chassis.waitUntilDone();*/
	Skills();
	//chassis.setPose(0,0,0);
	//chassis.moveToPose(0, 48, 0, 10000);
	/*frontstack.move_voltage(0);
	backstack.move_voltage(0);
	topstack.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.move_voltage(0);
	printf("done");*/
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
	

	descorer.set_value(HIGH);
	descorerState = false;
	
	//chassis.turnToHeading(90, 1000);

	while(true){
		
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

		//TOPSTAGE
		if(master.get_digital_new_press(DIGITAL_X))
		{
			if(IntakeState == 1)
			{
				intake1.move_velocity(600);
				intake2.move_velocity(600);
				IntakeState = 0;
			}
			else
			{
				intake1.move_velocity(0);
				intake2.move_velocity(0);
				IntakeState = 1;
			}
			printf("Intake state=%d intake velocity=%f \n", IntakeState, intake1.get_actual_velocity());
		}


		//REVERSE INTAKE
		if(master.get_digital_new_press(DIGITAL_B))
		{
			if(IntakeState == 3)
			{
				intake1.move_velocity(0);
				intake2.move_velocity(0);
				IntakeState = 0;
			}
			else
			{
				intake1.move_velocity(600);
				intake2.move_velocity(600);
				IntakeState = 3;
			}			
			printf("Intake state=%d intake velocity=%f \n", IntakeState, intake1.get_actual_velocity());
		}



		//MATCHLOADER
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
		{
			if(matchloaderState == true)
			{
				matchloader.set_value(LOW);
				matchloaderState = false;
			}
			else if(matchloaderState == false)
			{
				matchloader.set_value(HIGH);
				matchloaderState = true;
			}
			printf("Expansion state=%d \n", matchloaderState);
		}
		


		//Descorer
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
		{
			if(descorerState == true)
			{
				descorer.set_value(LOW);
				descorerState = false;
			}
			else
			{
				descorer.set_value(HIGH);
				descorerState = true;
			}
			printf("Expansion state=%d \n", matchloaderState);
		}
		printf("x: %f, y: %f, theta: %f \n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
		//printf("X: %f\n ", chassis.getPose().theta);
		pros::delay(10);
	};

}