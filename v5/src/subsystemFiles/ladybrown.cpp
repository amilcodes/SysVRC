#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystemHeaders/globals.hpp"
#include "main.h"
#include <string>
double RESTANGLE = 0; // actual -30

// Propped
double STOP1 = 26 + 1; // 42.57
// Semiextended
double STOP1_5 = STOP1 + 45;
// DescoreExtended
double STOP1_75 = STOP1 + 110 + 20;
// Extended
double STOP2 = 190 - 23 + 10; // angle of stop 2 - 130
// Almostfullextended + 20, Fullextended
double STOP3 = 190 + 1 + 3;

// double STOP1_75 = STOP1 + 110 - 2;
// double STOP2 = 190 - 45 + 15; // angle of stop 2 - 130
// double STOP3 = 250 - 60 + 1;


bool calledMoveBackForAWS = false;

double REST = 0;
double PROPPED = 1;
double EXTENDED = 2;
double FULLEXTENDED = 3;
double SEMIEXTENDED = 1.5;
double DESCOREEXTENDED = 1.75;
double ALMOSTFULLEXTENDED = 2.8;
double LBState = REST;

double LBAutonGoal = REST;
double prevLBAutonGoal = REST;
bool calledLBReset = false;

bool LBLoopActive = false;
long pressTime = 0;
long totalPressTime = 0;
bool lastPressed = false;

bool intakeUnstuckActivated = false;

int intakeStuckTime = 0;

long panicPressTime = 0;

/**
 * ONLY supposed to be used when intaking full mogo and hooks get caught
 */
void doIntakeUnstuck() {
    if (fabs(intake.get_actual_velocity()) < 2 && fabs(intake.get_voltage()) > 2000) { // if intake is stuck
        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        } else if (pros::millis() - intakeStuckTime > 400 && LBState == PROPPED) { // ring caught on ladybrown, extend a little
            intake.move(0);
            wrongColorDetected = true;
            LBExtend(SEMIEXTENDED);
            if (pros::competition::is_autonomous()) {
                intake.move(127); // restart intake if autonomous running
            }
            wrongColorDetected = false;
        }
        else if (pros::millis() - intakeStuckTime > 300 && LBState != PROPPED) {
            master.rumble("-"); // short rumble to notify driver
            double intakePower = intake.get_power();
            wrongColorDetected = true;
            intake.move(-127);
            pros::delay(400);
            intake.move(127);
            wrongColorDetected = false;
            intakeStuckTime = 0;
        }
        
    } else {
        intakeStuckTime = 0;
    }
}

/**
 * 
 */
void checkLBBroken() {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // buttons are all pressed
        if (panicPressTime == 0) {
            panicPressTime = pros::millis(); // start counting
        }
    } else if (panicPressTime != 0) { // all four buttons aren't being pressed and were pressed
        panicPressTime = 0; // reset
    }
    if (pros::millis() - panicPressTime > 500 && panicPressTime != 0) { // held for 0.5 seconds
        std::cout << "Ladybrown Emergency Activated" << "\n";
        LBRetract();
        pros::Task lb_task(LBLoop);
        LBLoopActive = true;
        LBState = REST;
        //LBRotation.set_position(0);
        ladybrown2.tare_position();
        panicPressTime = pros::millis();
    }
}

void tempFunction(double state, double stop, 
                  double curAng, 
                  double degreeOne, double degreeTwo, 
                  double moveOne, double moveTwo, double moveThree) {
    if(LBState == state)
    {
        if(stop - curAng > degreeOne)
        {
            ladybrown1.move(moveOne);
            ladybrown2.move(moveOne);
        }
        else if(stop - curAng < degreeTwo)
        {
            ladybrown1.move(moveTwo);
            ladybrown2.move(moveTwo);
        }
        else
        {
            ladybrown1.move(moveThree);
            ladybrown2.move(moveThree);
        }
    }
}


void doLBAmbientAdjust(double curAngle) {
    tempFunction(PROPPED, STOP1, curAngle, 0.5, -0.5, 8, -2.5, 1);
    tempFunction(SEMIEXTENDED, STOP1_5, curAngle, 10, -10, 13, -4, 2);
    tempFunction(EXTENDED, STOP2, curAngle, 5, -10, 10, -5, 0);

}



void LBExtend(double point) {
    double GOALANGLE;
    double power;
    double negPower;
    double angleChange;
    double iterationsRequired;
    const double kP = 6 - 1;
    const double kI = 0;
    const double kD = 0.02; 
    double totalError = 0;

    //double curAngle = -LBRotation.get_position() / 100.0;
    double curAngle = ladybrown2.get_position() / 3.0;
    int timeOut = 2000;

    if (point == 1) {
        GOALANGLE = STOP1;
        power = 70 * 0.01;
        if (curAngle > GOALANGLE) { // over and going back
            negPower = -50;
        } else {
            negPower = -5;
        }
        iterationsRequired = 40 - 20;
        angleChange = STOP1 - 0;
        timeOut = 1500;
    } else if (point == 2) {
        GOALANGLE = STOP2;
        power = 127;
        negPower = -10;
        angleChange = STOP2 - STOP1;
        iterationsRequired = 1;
        timeOut = 2500 + 2000;
    } else if (point > 2) {
        if (point == 2.8) {
            GOALANGLE = STOP3 - 20;
        } else if (point == 3) {
            GOALANGLE = STOP3;
        }
        power = 90;
        negPower = -15;
        angleChange = STOP3 - STOP2;
        iterationsRequired = 1;
        timeOut = 2500;
    } else if (point == 1.5) {
        GOALANGLE = STOP1_5;
        power = 70;
        negPower = -5;
        angleChange = STOP1_5 - STOP1;
        iterationsRequired = 1;
        timeOut = 1500;
    } else if (point == 1.75) {
        GOALANGLE = STOP1_75;
        power = 70;
        negPower = -8;
        angleChange = STOP1_5 - STOP1;
        iterationsRequired = 1;
        timeOut = 2000;
    }

    long startTime = pros::millis();
    double timeStayedGood = 0; // time stayed within range

    std::cout << "Extending to point " << point << ", Goal Angle: " << GOALANGLE << "\n";
    
    ladybrown1.move(power);
    ladybrown2.move(power);
    
    while ((abs(GOALANGLE - curAngle) > 3 || timeStayedGood < iterationsRequired) && pros::millis() - startTime < timeOut) { // ends once above goal angle
        //curAngle = -LBRotation.get_position() / 100.0;
        curAngle = ladybrown2.get_position() / 3.0;
        //std::cout << "Current Angle: " << curAngle << "\n";
        if (curAngle > GOALANGLE) {
            if (point == 1) {
                totalError += abs(GOALANGLE - curAngle);
                double total = abs(GOALANGLE - curAngle) * kP;
                if (total > 127) {
                    total = 127;
                }
                ladybrown1.move(-total);
                ladybrown2.move(-total);
            } else {
                ladybrown1.move(negPower);
                ladybrown2.move(negPower);
            }
        } else {
            if (point == 1) {
                totalError += abs(GOALANGLE - curAngle);
                double total = abs(GOALANGLE - curAngle) * kP + totalError * kI - ladybrown2.get_actual_velocity() * kD;
                if (total > 127) {
                    total = 127;
                }
                if (total < 0) {
                    total = 0;
                }
                ladybrown1.move(total);
                ladybrown2.move(total);
                //std::cout << "total: " + std::to_string(total) + "\n";
            } else {
                ladybrown1.move(power);
                ladybrown2.move(power);
            }
        }
        if (abs(GOALANGLE - curAngle) < 3) {
            timeStayedGood += 1;
        } else {
            timeStayedGood = 0;
        }
        if (curAngle > GOALANGLE && point > 1) {
            break;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2) && point == EXTENDED && curAngle < STOP1_5) { // if L2 pressed again and before stopping point 1.5
            // Switch to only extend up to point 1.5
            point = SEMIEXTENDED;
            GOALANGLE = STOP1_5;
            power = 70;
            negPower = -5;
            angleChange = STOP1_5 - STOP1;
            iterationsRequired = 1;
        }
        if (prevLBAutonGoal != LBAutonGoal) { // auton cancel lb
            prevLBAutonGoal = LBAutonGoal;
            ChangeLBAuton(LBAutonGoal);
            return;
        }
        pros::delay(10);
    }
    std::cout << "Reached Goal Angle: " << curAngle << "\n";
    ladybrown1.move(0); // stop once done
    ladybrown2.move(0); // stop once done
    //stateSetter(point);
    LBState = point;
    LBAutonGoal = point;
    prevLBAutonGoal = point;
}

void stateSetter(double point) {
    switch (static_cast<int>(point)) {
        case 1:
            LBState = PROPPED;
            break;
        case 2:
            LBState = EXTENDED;
            break;
        case 3:
            LBState = FULLEXTENDED;
            break;
        case 4:
            LBState = SEMIEXTENDED;
            break;
        default:
            break;
    }
}

/**
 * @brief Retracts ladybrown to rest angle
 * 
 */
void LBRetract() {
    ladybrown1.move(-127); // move back
    ladybrown2.move(-127);
    pros::delay(200);
    long startTime = pros::millis();
    //double prevAngle = -LBRotation.get_position() / 100.0;
    double prevAngle = ladybrown2.get_position() / 3.0;
    double curAngle = prevAngle;
    double power;
    while (curAngle > 25 && pros::millis() - startTime < 1500) { // wait for motors to stop
        power = curAngle + 5;
        if (power > 127) {
            power = 127;
        }
        ladybrown1.move(-power);
        ladybrown2.move(-power);
        //curAngle = -LBRotation.get_position() / 100.0;
        curAngle = ladybrown2.get_position() / 3.0;
        pros::delay(10);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            ladybrown1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            ladybrown2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            LBExtend(PROPPED);
            return;
        }
    }
    while (fabs(ladybrown2.get_actual_velocity()) > 1 && pros::millis() - startTime < 2000) {
        ladybrown1.move(-40 + 30);
        ladybrown2.move(-40 + 30);
        pros::delay(10);
    }
    ladybrown1.move(0);
    ladybrown2.move(0);
    pros::delay(100);
    LBState = REST;
    LBAutonGoal = REST;
    prevLBAutonGoal = REST;
    LBRotation.reset_position();
    ladybrown2.tare_position();
    ladybrown1.tare_position();
    //pros::delay(2000);
}

void ChangeLBState(double goal) {
    LBAutonGoal = goal;
}

/**
 * @brief Changes the ladybrown to a certain state
 * @param goal the goal to change to
 */
void ChangeLBAuton(double goal) {
    if (goal == REST) {
        std::cout << "New goal: " << goal << "\n";
        LBRetract();
    } else {
        LBExtend(goal);
    }
}

void callLBReset() {
    calledLBReset = true;
}

/**
 * @brief main ladybrown task loop
 * 
 */
void LBLoop() {
    LBLoopActive = true;
    ladybrown1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ladybrown2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while (true) {
        ladybrown1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        ladybrown2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        //double curAngle = -LBRotation.get_position() / 100.0;
        double curAngle = ladybrown2.get_position() / 3.0;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // IMPORTANT: must be new_press
            if (!lastPressed) { // just pressed
                pressTime = pros::millis();
                totalPressTime = pros::millis();
            }
            if (pros::millis() - pressTime > 750) { // held for 0.75 seconds
                LBRetract();
                pressTime = pros::millis();
            }
            lastPressed = true;
        } else {
            if (lastPressed) { // just released
                if (pros::millis() - totalPressTime > 500) { // held for 0.5 seconds
                    LBRetract();
                } else { // pressed for normal logic
                    
                    //std::cout << "Button L2 pressed, Current Angle: " << curAngle << "\n";
                    if (curAngle < STOP1 - 10) { // at stopping point 1
                        std::cout << "At rest, extending to point 1\n";
                        LBExtend(1); // go to stopping point 2
                    } else if ((curAngle < STOP2 - 5) && LBState != EXTENDED && LBState != DESCOREEXTENDED) { // at 1.5
                        std::cout << "At stopping point 1, going to stopping point 2\n";
                        LBExtend(2); // go to rest
                    } else { // at rest
                        std::cout << "At EXTENDED, going to rest\n";
                        stopDriverIntake = false;
                        LBRetract(); // go to stopping point 1
                    }
                }
            }
            lastPressed = false;

        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            LBExtend(3);
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            LBExtend(DESCOREEXTENDED);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            // chassis.pid_drive_set(-8, 127); // move back
            // chassis.pid_wait();
            // LBExtend(3);
            calledMoveBackForAWS = true;
        }
        if (LBAutonGoal != prevLBAutonGoal) { // interact with LB in auton mode
            prevLBAutonGoal = LBAutonGoal;
            ChangeLBAuton(LBAutonGoal);
        }
        if (calledLBReset) {
            LBRetract();
            calledLBReset = false;
        }
        prevLBAutonGoal = LBAutonGoal;

        // Ambiend Adjust here
        doLBAmbientAdjust(curAngle);
        if (intakeUnstuckActivated) {
            doIntakeUnstuck();
        }
        pros::delay(20);
    }
}

void TwoRingLBMacro() {
    intake.move(110);
    startColorUntil(1);
    LBExtend(2);
    set_drive(-8, 1000);
    chassis.pid_wait();
    set_drive(8, 1000);
    LBRetract();
    chassis.pid_wait();
    LBExtend(2);
    set_drive(-8, 1000);
    chassis.pid_wait();
    stopColorUntilFunction();
}