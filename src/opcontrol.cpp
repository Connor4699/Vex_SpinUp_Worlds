#include "main.h"

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



void my_opcontrol() {
	while (true) {
        VoltageControl();
        op_drive();


        pros::delay(10);
		pros::delay(20);
	}
}