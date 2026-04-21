#include "main.h"
#include "pros/misc.h"

auto liftButton = pros::E_CONTROLLER_DIGITAL_A;
bool wrongColorDetected;
bool stopDriverIntake = false;
bool manualLadybrownActivated = false;

// Helper functions
void setIntake(int power) {
    // Sets the intake motor velocity directly using PROS motor control
    // Positive velocity = intake in, Negative velocity = intake out
    // velocity: target velocity in RPM (-600 to 600)
    intake.move(power);
}

// Driver Control Functions
void setIntakeMotors() {
    // Function handles intake control during driver control period
    // Uses velocity-based control for consistent intake power
    // Controls:
    //   - R2: Intake inward (collect game elements) at 600 RPM
    //   - R1: Outtake/reverse (release game elements) at -600 RPM
    //   - Neither pressed: Stop intake (0 RPM)
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // new double press at same time
        intake.move(0);
        manualLadybrownActivated = !manualLadybrownActivated;
    }

    
    
    int intakePower = 0;  // Default state: stopped

    if (stopDriverIntake && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        stopDriverIntake = false;
    }
    
    // Priority system: if both buttons pressed, R1 takes priority
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // TODO: took out stopDriverIntake
        intakePower = INTAKE_POWER;     // Full speed intake
        if (manualLadybrownActivated) {
            intakePower = 70;
        } else if (LBState == PROPPED || LBState == SEMIEXTENDED) {
            intakePower = 110;
        }

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        intakePower = -INTAKE_POWER;    // Full speed outtake
    }
    // If neither button is pressed, intakeVelocity remains 0
    
    // Apply the calculated velocity to the intake motor
    if (manualLadybrownActivated) {
        ladybrown1.move(intakePower);
        ladybrown2.move(intakePower);
    } else {
        if (!wrongColorDetected) { // if color sort hasn't activated
            setIntake(intakePower);
        }
    }
    
    
}


// Driver Control Functions
void setIntakeLift() {
    if (master.get_digital(liftButton)) { // button pressed
        intakeLift.extend();
    } else {
        intakeLift.retract();
    }
}