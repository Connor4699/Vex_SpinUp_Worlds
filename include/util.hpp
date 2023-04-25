#include <iostream>

// Screen Tasks
void log(std::string name);
void screen();

// Motor Functions
void setDrive(double leftPower, double rightPower);
void setIntake(double power);

// Specific Task Functions
void getRoller();
void shootDisc(int discs, int del);

// Util Functions
void vector(float x, float y, std::string name = "", bool in = false);
void faceBlueNet();
void faceRedNet();

extern double autonDesired;
void AutonVoltageControl();


