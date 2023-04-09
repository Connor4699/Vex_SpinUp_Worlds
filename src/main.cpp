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
#include <fstream>
#include <ostream>
#include <string>

pros::ADIAnalogIn in_sensor('H');
pros::ADIAnalogIn out_sensor('B');
pros::ADILightSensor light_sensor('C');
pros::Motor lF(11, pros::E_MOTOR_GEARSET_06, true);
pros::Motor lM(12, pros::E_MOTOR_GEARSET_06, false); 
pros::Motor lB(13, pros::E_MOTOR_GEARSET_06, true); 
pros::Motor rF(18, pros::E_MOTOR_GEARSET_06, false); 
pros::Motor rM(19, pros::E_MOTOR_GEARSET_06, true); 
pros::Motor rB(20, pros::E_MOTOR_GEARSET_06, false); 

pros::Motor intake(2, pros::E_MOTOR_GEARSET_06, false);
pros::Motor flywheel(1, pros::E_MOTOR_GEARSET_06, false);

pros::MotorGroup leftMotors({lF, lM, lB});
pros::MotorGroup rightMotors({rF, rM, rB});

pros::Rotation left_rot(16, false);
pros::Rotation right_rot(15, false);
pros::Rotation back_rot(10, false);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Imu inertial_sensor(4);

lemlib::TrackingWheel left_tracking_wheel(&left_rot, 2.75, 3.125);
lemlib::TrackingWheel right_tracking_wheel(&right_rot, 2.75, -3.125);
lemlib::TrackingWheel back_tracking_wheel(&back_rot, 2.75, -6);


lemlib::Drivetrain_t drivetrain {
    &leftMotors, // left drivetrain motors
    &rightMotors, // right drivetrain motors
    12, // track width
    3.25, // wheel diameter
    400 // wheel rpm
};


// forward/backward PID
lemlib::ChassisController_t lateralController {
    30, // kP
    125, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    2, // kP
    20, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};


lemlib::OdomSensors_t sensors {
   // &left_tracking_wheel, // vertical tracking wheel 1
   // &right_tracking_wheel, // vertical tracking wheel 2
  //  &back_tracking_wheel, // horizontal tracking wheel 1
    .vertical1=&left_tracking_wheel,//&left_tracking_wheel
    nullptr,//&right_tracking_wheel
    &back_tracking_wheel,
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &inertial_sensor // inertial sensor
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);


void setDrive(double leftPower, double rightPower) {
    lF.move(leftPower);
    lB.move(leftPower);
    rF.move(rightPower);
    rB.move(rightPower);
    rM.move(rightPower);
    lM.move(leftPower);
}

void op_drive() {
    const int deadband = 5;
    int x = abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    int y = abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    double LjoyY = y/10.0;
    double LjoyX = x/10.0;
    double power = 0;
    double turn = 0;

    if (y > deadband || x > deadband) {
        if (y > 85) {
            power = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyY - 6.0)))) * 10.0 - 3.0;
        }
        else if (y > 55 && y <= 85) {
            power = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyY - 6.0)))) * 10.0 - 10.0;
        }
        else {
            power = 5*pow((1.0/5.5)*(LjoyY), 3.0) * 12.7;
        }

        if (x > 95) {
            turn = 0.8*(12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyX - 6.0)))) * 10.0 - 3.0;
        }
        else if (x > 55 && x <= 85) {
            turn = 0.35*(12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyX - 6.0)))) * 10.0 - 10.0;
        }
        else {
            turn = 0.6*5*pow((1.0/5.5)*(LjoyX), 3.0) * 12.7;
        }
    }

    if (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0) {
        power = -power;
    }

    if (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) < 0) {
        turn = -turn;
    }

    double leftPower = (power + turn);
    double rightPower = (power - turn);

    lF.move(leftPower);
    lB.move(leftPower);
    rF.move(rightPower);
    rB.move(rightPower);
    rM.move(rightPower);
    lM.move(leftPower);
}

void setIntake(double power) {
    intake.move(power);
}

void getRoller() {
    setDrive(-50, -50);
    setIntake(-127);
    pros::delay(200);
    setDrive(50, 50);
    pros::delay(200);
    setDrive(0, 0);
    setIntake(0);
}

void shootDisc() {

}

double autonDesired = 400;
void AutonVoltageControl() {
    while (true) {
        double max = 0;
        double finalVolt = 0;
        double flywheel_speed = flywheel.get_actual_velocity();
        
        if (flywheel_speed < autonDesired) {
            max = 12000;
        } 
        else {
            max = (autonDesired/600 * 12000) - 100;
        }

        finalVolt = max;
        if (finalVolt > 12000) {
            finalVolt = 12000;
        }
        flywheel.move_voltage(finalVolt);

        pros::lcd::set_text(1, "Flywheel Velocity: " + std::to_string(flywheel_speed));
        pros::lcd::set_text(2, "Flywheel Voltage: " + std::to_string(flywheel.get_voltage()));
        
        pros::delay(10);
    }
}



double volt = 8000;
double desired = 410;
void VoltageControl() {
    double max = 0;
    double finalVolt = 0;
    double flywheel_speed = flywheel.get_actual_velocity();
    
    if (flywheel_speed < desired) {
        max = 12000;
    } 
    else {
        max = desired*20;
    }

    if (controller.get_digital_new_press(DIGITAL_Y)) {
        desired = 380;
    }
    else if (controller.get_digital_new_press(DIGITAL_A)) {
        desired = 410;
    }
    else if (controller.get_digital_new_press(DIGITAL_X)) {
        desired += 10;
    }
    else if (controller.get_digital_new_press(DIGITAL_B)) {
        desired -= 10;
    }
    finalVolt = max;
    if (finalVolt > 12000) {
        finalVolt = 12000;
    }
    flywheel.move_voltage(finalVolt);

    pros::lcd::set_text(4, "Flywheel Velocity: " + std::to_string(flywheel_speed));
    pros::lcd::set_text(5, "Flywheel Voltage: " + std::to_string(flywheel.get_voltage()));
}





 void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}


void log(std::string name = "") {
    lemlib::Pose pose = chassis.getPose();
    std::ofstream Data;
    Data.open("/usd/Logs/log.txt", std::ios_base::app);
    Data << "NAME: " + name << std::endl;
    Data << "x: " + std::to_string(pose.x) << std::endl;
    Data << "y: " + std::to_string(pose.y) << std::endl;
    Data << "angle: " + std::to_string(pose.theta) << std::endl;
    Data << "---------------------------" << std::endl;
    Data.close();
}

void vector(float x, float y, std::string name = "", bool in = false) {
    if (in == true) {
        setIntake(127);
    }
    chassis.turnTo(x, y, 2000);
    chassis.moveTo(x, y, 2000);
    log(name);
    setIntake(0);
}

void faceRedNet() {
    chassis.turnTo(-53, -53, 1000);
}

void faceBlueNet() {
    chassis.turnTo(53, 53, 1000);
}

void right_auton() {
    chassis.setPose(17, -53, 0);
    vector(35, -35);
    //intake
    chassis.turnTo(36, -72, 1000, true);
    chassis.moveTo(35, -64, 2000);
    // get roller
    chassis.moveTo(36, -60, 1000);
    chassis.turnTo(53, 53, 1000);
    // shoot
    vector(-12, -12);
    chassis.turnTo(53, 53, 1000);
    // shoot
    vector(-20, -20);
    // intake
    vector(-20, -54);
    chassis.turnTo(53, 53, 1000);
    // shoot
}

void left_auton() {
    chassis.setPose(-64, 40, 90);
    vector(-48, 48);

}

void skills_auton() {

    chassis.setPose(63, -42, -90);
    log("Skills");
    getRoller();
    vector(48, -48, "1st intake", true);
    vector(40, -60);
    vector(24, -48, "1st shot");
    faceRedNet();
    // shoot
    // vector(-12, -12, "2nd intake", true);
    // faceRedNet();
    // shoot



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
void autonomous() {
    pros::Task fwc(AutonVoltageControl);
    
    skills_auton();
    fwc.suspend();


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
        my_opcontrol()
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) == 1) {
        //     flywheel.move_voltage(12000);
        // }
    }


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