#include "main.h"
#include "pros/rtos.hpp"
#include <iostream>
#include <ostream>
#include <fstream>

// Screen Tasks
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

// Motor Functions
void setDrive(double leftPower, double rightPower) {
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


// Specific Task Functions

void getRoller() {
    setDrive(-50, -50);
    setIntake(127);
    pros::delay(500);
    setIntake(0);
    setDrive(50, 50);
    pros::delay(200);
    setDrive(0, 0);
}

void shootDisc(int discs, int del) {
    int i = 0;
    while (i < discs) {
        intake.move(-110);
        pros::delay(150);
        intake.move(0);
        i = i + 1;
        if (i == discs) {
            pros::delay(150);
            break;
        }
        pros::delay(del);
    }
}


// Util Functions
void vector(float x, float y, std::string name, bool in) {
    if (in == true) {
        setIntake(127);
    }
    chassis.turnTo(x, y, 800);
    chassis.moveTo(x, y, 1600);
    log(name);
    setIntake(0);
}

void faceRedNet() {
    chassis.turnTo(-53, -53, 1000);
}

void faceBlueNet() {
    chassis.turnTo(57, 58, 1300);
}


double autonDesired = 550;
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
