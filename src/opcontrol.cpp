#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"

void op_intake() {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == 1) {
            intake.move_voltage(11000 * controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1));
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == 1) {
            intake.move_voltage(-11000 * controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2));
        }

        else {
            intake.move_voltage(0);
        }
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

double volt = 8000;
double desired = 390;
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
        desired = 390;
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


void my_opcontrol() {
    int a = 1;
    int b = 1;
    int c = 1;
    int d = 1;
	while (true) {

        //  pros::lcd::set_text(3, "d: " + std::to_string(a));
        // pros::lcd::set_text(4, "e: " + std::to_string(b));
        // pros::lcd::set_text(5, "g: " + std::to_string(c)); 
        // pros::lcd::set_text(6, "f: " + std::to_string(d)); 


        VoltageControl();
        op_drive();
        op_intake();
        controller.set_text(1, 1, "FV: " + std::to_string(flywheel.get_actual_velocity()));
        pros::delay(10);

        
		if (controller.get_digital(DIGITAL_L1)) {
			angler.set_value(true);
		}
		if (controller.get_digital(DIGITAL_L2)) {
			angler.set_value(false);
		}

        if (controller.get_digital(DIGITAL_A)) {
			intake_lift.set_value(false);
		}
		if (controller.get_digital(DIGITAL_Y)) {
			intake_lift.set_value(true);
		}



        //endgame

        if (controller.get_digital(DIGITAL_LEFT)) {
            Right_valve.set_value(true);
            Bottom_endgame.set_value(true);
        }

        if (controller.get_digital(DIGITAL_UP)) {
            Left_release.set_value(true);
            Bottom_endgame.set_value(true);
        }

        if (controller.get_digital(DIGITAL_DOWN)) {
            Bottom_endgame.set_value(true);
        }

        //endgame loading
        // if (controller.get_digital_new_press(DIGITAL_B)) {
        //     if (a == 1) {
        //         Left_release.set_value(false);
        //         a = 0;
        //     } else {
        //         Left_release.set_value(true);
        //         a = 1;
        //     }         
        // }
        // else if (controller.get_digital_new_press(DIGITAL_A)) {
        //     if (b == 1) {
        //         Right_release.set_value(false);
        //         b = 0;
        //     } else {
        //         Right_release.set_value(true);
        //         b = 1;
        //     }
        // }
        // else if (controller.get_digital_new_press(DIGITAL_Y)) {
        //     if (c == 1) {
        //         Right_valve.set_value(false);
        //         c = 0;
        //     } else {
        //         Right_valve.set_value(true);
        //         c = 1;
        //     }
        // }
        // else if (controller.get_digital_new_press(DIGITAL_X)) {
        //     if (d == 1) {
        //         Left_valve.set_value(false);
        //         d = 0;
        //     } else {
        //         Left_valve.set_value(true);
        //         d = 1;
        //     }
        // }
	}
}