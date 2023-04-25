#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"

// Sensors
pros::ADIAnalogIn in_sensor('H');
pros::ADIAnalogIn out_sensor('G');
// pros::ADILightSensor light_sensor('C');

pros::Imu inertial_sensor(4);

// Encoders
pros::Rotation left_rot(16, false);
pros::Rotation right_rot(15, false);
pros::Rotation back_rot(10, false);

// Motors
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

pros::Controller controller(pros::E_CONTROLLER_MASTER);
// Lemlib
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
    7, // kP
    20, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    3 // slew rate
};

// turning PID
lemlib::ChassisController_t angularController {
    5, // kP
    15, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    4 // slew rate
};

// // MOTOR ENCODERS
// lemlib::OdomSensors_t sensors {
//     nullptr,//&left_tracking_wheel
//     nullptr,//&right_tracking_wheel
//     nullptr,
//     nullptr, // we don't have a second tracking wheel, so we set it to nullptr
//     &inertial_sensor // inertial sensor
// };


lemlib::OdomSensors_t sensors {
    &left_tracking_wheel,//&left_tracking_wheel
    nullptr,//&right_tracking_wheel
    nullptr,
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &inertial_sensor // inertial sensor
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

pros::ADIDigitalOut angler('A');
pros::ADIDigitalOut intake_lift('B');

//Endgame
pros::ADIDigitalOut Left_release('D');
pros::ADIDigitalOut Right_release('E');
pros::ADIDigitalOut Left_valve('F');
pros::ADIDigitalOut Right_valve('G');
pros::ADIDigitalOut Bottom_endgame('H');
