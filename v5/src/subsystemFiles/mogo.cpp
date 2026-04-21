#include "main.h"

using namespace std;
auto mogoTrigger = pros::E_CONTROLLER_DIGITAL_L1;

const double AUTOCLAMP_DISTANCEREACTIVATE = 50;

const double AUTOCLAMP_DISTANCE = 10;

bool autoClampActivated = false;

bool tempDisableAutoclamp = false;

void clampMogo(bool active) {
    mogoClamp.toggle();
}

// Driver Control Functions
void setMogoMotors() {
    double curDistance = autoClampSensor.get_distance();
    if (!mogoClamp.is_extended() && curDistance < AUTOCLAMP_DISTANCE && autoClampActivated && !tempDisableAutoclamp) { // unclamped and ready to auto clamp
        mogoClamp.extend();
        tempDisableAutoclamp = true;
    }
    if (master.get_digital_new_press(mogoTrigger)) { // button pressed 
        mogoClamp.retract();
    }
    if (!master.get_digital(mogoTrigger)) {
        mogoClamp.extend();
        tempDisableAutoclamp = true;
    }
    if (curDistance > AUTOCLAMP_DISTANCEREACTIVATE) {
        tempDisableAutoclamp = false;
    }
}