#include "main.h"

using namespace std;

void toggleDoinker(bool active) {
    rightDoinker.toggle();
}

void setDoinker() {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        rightDoinker.toggle();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        leftDoinker.toggle();
    }
}