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

// Sensors
extern pros::ADIAnalogIn in_sensor;
extern pros::ADIAnalogIn out_sensor;
extern pros::ADILightSensor light_sensor;
extern pros::Imu inertial_sensor;

// Encoders
extern pros::Rotation left_rot;
extern pros::Rotation right_rot;
extern pros::Rotation back_rot;

// Motors
extern pros::Motor rB;
extern pros::Motor rM;
extern pros::Motor rF;
extern pros::Motor lB;
extern pros::Motor lM;
extern pros::Motor lF;

extern pros::Motor intake;
extern pros::Motor flywheel;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// Controller
extern pros::Controller controller;

// LemLib
extern lemlib::TrackingWheel left_tracking_wheel;
extern lemlib::TrackingWheel right_tracking_wheel;
extern lemlib::TrackingWheel back_tracking_wheel;

extern lemlib::Drivetrain_t drivetrain;
extern lemlib::ChassisController_t lateralController;
extern lemlib::ChassisController_t angularController;
extern lemlib::OdomSensors_t sensors;
extern lemlib::Chassis chassis;

// Pneumatics
extern pros::ADIDigitalOut angler;
extern pros::ADIDigitalOut intake_lift;

extern pros::ADIDigitalOut Left_release;
extern pros::ADIDigitalOut Right_release;
extern pros::ADIDigitalOut Left_valve;
extern pros::ADIDigitalOut Right_valve;
extern pros::ADIDigitalOut Bottom_endgame;
