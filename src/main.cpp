#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "autoSelect/selection.h"
#include <fstream>
#include <ostream>
#include <string>


void faceTest() {
    chassis.turnTo(45, 53, 1200);
}

void right_auton() {
    chassis.setPose(17.5, -55.75, 90);
    autonDesired = 590;
    vector(33, -55.75); //drive toward roller
    chassis.turnTo(33, -72, 750, true);
    chassis.moveTo(33, -65, 800);
    intake.move(127);
    pros::delay(200);
    chassis.moveTo(24, -48, 750);
    // intake.move(127);
    // pros::delay(1800);
    // intake.move(0);
    faceBlueNet();
    shootDisc(2, 400);
    pros::delay(100);

    autonDesired = 550;
    chassis.turnTo(35, -35, 700);
    intake_lift.set_value(true);
    setIntake(120);
    chassis.turnTo(33.5, -39, 800);
    chassis.moveTo(33.5, -39, 1000);
    // vector(31, -39, "", true);
    // chassis.moveTo(34, -40, 1000);
    intake_lift.set_value(false);
    pros::delay(900);
    // vector(33, -37);
    // pros::delay(500)
    chassis.moveTo(24, -48, 750);
    chassis.turnTo(61, 58, 1000);
    // faceBlueNet();
    setIntake(0);
    // faceBlueNet();
    shootDisc(3, 400);

    autonDesired = 540;
    vector(0, -24, "", true);
    setIntake(127);
    pros::delay(250);
    chassis.turnTo(74, 65, 1600, false, 95);
    setIntake(0);
    shootDisc(4, 300);

}

void left_auton() {
    chassis.setPose(-62.75, 37.5, 90);
    autonDesired = 600;
    getRoller();
    setIntake(110);
    vector(-51, 43, "", true);
    setIntake(110);
    chassis.moveTo(-48, 24, 1200);
    chassis.turnTo(63.5, 51, 1200);
    // faceTest();
    setIntake(0);
    shootDisc(3, 350);
    
    autonDesired = 565;
    chassis.turnTo(-42, 32, 800);
    setIntake(127);
    intake_lift.set_value(true);
    chassis.moveTo(-42, 32, 1200);
    intake_lift.set_value(false);
    pros::delay(1000);
    chassis.moveTo(-48, 24, 1200);
    chassis.turnTo(53.5, 54, 1300);
    // faceTest();
    setIntake(0);
    shootDisc(3, 270);


    autonDesired = 565;
    intake_lift.set_value(true);
    vector(-41, 17);
    intake_lift.set_value(false);
    setIntake(120);
    pros::delay(1300);
    chassis.turnTo(40.5, 63, 1250);
    // faceTest();
    setIntake(0);
    shootDisc(5, 250);


    // chassis.turnTo(-55, -18, 1000, true);
    // chassis.moveTo(-55, -18, 3000);
    // vector(-26, -18, "Intake 3 near low goal", true);
    // chassis.moveTo(-36, -12, 2000);
    // faceBlueNet();
    // shootDisc(3, 200);
}

void double_roller() {
    chassis.setPose(-62.25, 29.5, 90);
    autonDesired = 600;
    setDrive(-50, -50);
    setIntake(-127);
    pros::delay(200);
    setIntake(0);
    setDrive(50, 50);
    pros::delay(200);
    setDrive(0, 0);
    
    // faceBlueNet();
    // shootDisc(3, 300);
    
    autonDesired = 560;
    chassis.moveTo(-48, 24, 800);
    setIntake(120);
    intake_lift.set_value(true);
    setIntake(120);
    chassis.turnTo(-42, 20, 750);
    chassis.moveTo(-42, 20, 1000);
    // vector(-42, 20, "", true);
    setIntake(120);
    intake_lift.set_value(false);
    chassis.moveTo(-24, 0, 1500, 75);
    pros::delay(250);
    chassis.turnTo(52, 62, 1600);
    // faceTest();
    setIntake(0);
    shootDisc(3, 400);

    autonDesired = 560;
    setIntake(127);
    chassis.turnTo(12, -36, 1000);
    chassis.moveTo(12, -36, 2500, 80);
    pros::delay(1000);
    chassis.turnTo(51, 63, 1700);
    // faceTest();
    setIntake(0);
    shootDisc(3, 350);

    // Get other roller
    chassis.moveTo(31, -61, 1200);
    chassis.turnTo(31, -72, 1000, true);
    chassis.moveTo(31, -66, 800);
    intake.move(-127);
    pros::delay(200);
    intake.move(0);


}

void skills_auton() {

    chassis.setPose(63, -42, -90);
    log("Skills");
    getRoller();
    vector(48, -48, "1st intake", true);
    chassis.turnTo(48, -72, 1000, true);
    chassis.moveTo(48, -65, 2000);
    getRoller();

    autonDesired = 500;
    vector(24, -48, "1st shot");
    faceRedNet();
    shootDisc(3, 200);

    autonDesired = 450;
    vector(-12, -12, "2nd intake", true);
    faceRedNet();
    shootDisc(3, 200);



}

int total = 0;
bool in_cur = false;
bool out_cur = false;
bool dir_in = false;
bool dir_out = false;
void disctrack() {
    while (true) {
        int disc_value = 2700;
        int emp_value = 2800;

        if (total < 0) {
            total = 0;
        }

        if (total >= 5) {
            pros::lcd::set_text(1, "FULL");
            controller.rumble("-");
        }

        if (in_sensor.get_value() < disc_value) {
            if (intake.get_actual_velocity() > 0) {
                dir_in = true;
            }
            else {
                dir_in = false;
            }
            in_cur = true;
        }

        if (in_cur == true) {
            
            if (in_sensor.get_value() > emp_value) {
               
                total = total + 1;
                in_cur = false;
            }
            if (intake.get_actual_velocity() < -1) {
                if (in_sensor.get_value() > emp_value) {
                    total -= 1;
                    in_cur = false;
                }

            }
        }

        // if (out_sensor.get_value() < disc_value) {
        //     out_cur = true;
        // }

        // if (out_cur == true && out_sensor.get_value() > emp_value) {
        //     total -= 1;
        //     out_cur = false;
        // } 
        // pros::lcd::set_text(2, "Light Sensor: " + std::to_string(light_sensor.get_value()));
        pros::lcd::set_text(2, "In-Cur: " + std::to_string(in_cur));
        pros::lcd::set_text(3, "Discs: " + std::to_string(total-1));
        pros::lcd::set_text(4, "Line Sensor: " + std::to_string(in_sensor.get_value())); 
        pros::lcd::set_text(5, "Line Sensor 2: " + std::to_string(out_sensor.get_value())); 
        pros::delay(10);
    }
}

void initialize() {
	pros::lcd::initialize();
	chassis.calibrate(); // calibrate the chassis
	
	chassis.setPose(0, 0, 0);

    chassis.setPose(-62.75, 37.5, 90);
    // chassis.setPose(-62.75, 37.5, 90);
    // chassis.setPose(17.5, -55.75, 90);

    //selector::init();
    // pros::delay(2000);
    
	//chassis.setPose(63, -45, -90); // X: 0, Y: 0, Heading: 0
    pros::Task screenTask(screen); // create a task to print the position to the screen
    
    // pros::Task l(disctrack);
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
pros::Task fwc(AutonVoltageControl);

void autonomous() {
    
    // pros::Task fwc(AutonVoltageControl);
    double_roller();
    //right_auton();
    // right_auton();
    fwc.suspend();

    // if (selector::auton == 1) {
    //     left_auton();
    //     fwc.suspend();
    // }

    // if (selector::auton == 2) {
        
    //     right_auton();
    //     fwc.suspend();
    // }

    // if (selector::auton == 3) {
        
    //     double_roller();
    //     fwc.suspend();
    // }


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
    fwc.suspend();
    controller.clear();
    pros::delay(100);
    // std::ofstream Data;
    // Data.open("/usd/Logs/log.txt", std::ios_base::app);
    // double time = 0;    
    // Data << "Time (s), Velocity, Voltage (mV), Wattage (W)" << std::endl;
    // flywheel.move_voltage(12000);
    
	// while (time < 10.0) {
    //     Data << std::to_string(time) + ", " + std::to_string(flywheel.get_actual_velocity()) + ", " + std::to_string(flywheel.get_voltage()) + ", " + std::to_string(flywheel.get_power()) << std::endl;
    //     if (time == 5.0 || time == 5.4 || time == 5.8) {
    //         intake.move(-110);
    //         pros::delay(100);
    //         intake.move(0);
    //         time = time + 0.1;
    //     }
    //     pros::delay(10);
    //     time = time + 0.01;
    
    // }
    // flywheel.move_voltage(0);
    // Data << "END" << std::endl;
    // Data.close();
     my_opcontrol();
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) == 1) {
        //     flywheel.move_voltage(12000);
        // }
    

    // Data Logging
    // std::ofstream Data;
    // Data.open("/usd/Logs/log.txt", std::ios_base::app);
    // double time = 0;    
    // Data << "Time (s), Velocity, Voltage (mV), Wattage (W)" << std::endl;
    // flywheel.move_voltage(12000);
    
	// while (time < 10.0) {     
    //     Data << std::to_string(time) + ", " + std::to_string(flywheel.get_actual_velocity()) + ", " + std::to_string(flywheel.get_voltage()) + ", " + std::to_string(flywheel.get_power()) << std::endl;
    //     pros::delay(10);
    //     time = time + 0.01;
    
    // }
    // flywheel.move_voltage(0);
    // Data << "END" << std::endl;
    // Data.close();
}