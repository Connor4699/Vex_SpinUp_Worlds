#pragma once

#include <string>

//selector configuration
#define HUE 360
#define DEFAULT 5
#define AUTONS "Left", "Right", "Double Roller", "Safe Left", "Safe Right"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
