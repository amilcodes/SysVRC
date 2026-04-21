#include "pros/motors.h"
#include "main.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

void skills() {
    chassis.odom_look_ahead_set(10);

    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    chassis.odom_xyt_set(-60.5, -13, (-46)); // starts at middle of red alliance line
    //pros::Task lb_task(LBLoop);
    //LBState = EXTENDED;
    //LBRotation.set_position(STOP1);
    //ladybrown2.set_zero_position(-46);
    //ChangeLBState(FULLEXTENDED);
    intakeUnstuckActivated = true;
    ColorLoopActive = false;
    colorFiltrationActive = false;

    //pros::delay(200);
    //intake.move(-127);
    //pros::delay(300 - 50);
    intake.move(0);

    set_drive(-19.5 + 0.5, 1500, 0, 70); // move away from alliance stake
    chassis.pid_wait_until(-16);


    /////////////////////////// FIRST MOGO ///////////////////////////
    /////////////////////////// FIRST MOGO ///////////////////////////
    

    mogoClamp.toggle(); // clamp mogo
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown

    setIntake(127);
    intake.move(127);
    chassis.pid_turn_set(90, 90); // turn to two rings
    chassis.pid_wait();

    chassis.pid_drive_set(16 + 0.5, 110); // move to first ring
    chassis.pid_wait();
    chassis.pid_turn_set(116.5 - 1, 90); // turn to corner ring stack
    chassis.pid_wait();
    chassis.pid_drive_set(79 - 25 + 2, 127); // move to next two rings in corner 2 stack
    chassis.pid_wait_until(45 - 15);
    intake.move(100);
    ChangeLBState(PROPPED); // prop up ladybrown
    chassis.pid_wait();
    intake.move_voltage(12000);
    chassis.pid_turn_set(103 + 2, 90); // turn to go back
    chassis.pid_wait();
    intake.move_voltage(12000);
    chassis.pid_drive_set(-43 + 17.25 - 1, 110); // move back a bit
    chassis.pid_wait();
    chassis.pid_turn_set(180, 90); // turn to wall stake
    chassis.pid_wait();
    setIntake(0);
    ChangeLBState(SEMIEXTENDED);

    chassis.pid_drive_set(14 + 20, 70, false, false);
    chassis.pid_wait_until(5);
    chassis.pid_speed_max_set(55);
    intake.move(127);
    chassis.pid_wait_until(13.5 - 1.5);

    // chassis.pid_wait();

    ChangeLBState(EXTENDED); // extend ladybrown
    chassis.drive_set(127, 127);
    pros::delay(400 - 50);
    
    set_drive(-15 - 0.35); // go back a bit
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown

    chassis.pid_turn_set(-89.9 + 1, 90); // turn to two rings
    setIntake(127);
    chassis.pid_wait();
    intake.move_voltage(12000);

    // collecting 3 rings

    callLBReset();

    set_drive(58.5 + 1.5, 3000, 0, 70);  // cap the max speed at 70
    chassis.pid_wait_until(29);
    chassis.pid_speed_max_set(55); // intake rings slower
    chassis.pid_wait();
    setIntake(127);
    chassis.pid_turn_set(140 - 10 + 10, 60); // turn to last ring before corner
    chassis.pid_wait_quick_chain();
    setIntake(127);
    set_drive(13.5 + 2 - 2, 1500, 75, 120); // collect last bottom-left ring
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(75 - 5, 90 - 35); // turn to corner
    chassis.pid_wait();
    setIntake(127);

    set_drive(-17 - 1.5, 750, 0, 70); // move to corner
    chassis.pid_wait();
    //pros::delay(500);
    setIntake(0);
    mogoClamp.toggle(); // unclamp mogo

    set_drive(9 + 0.5, 1500, 60, 120); // move out of corner
    chassis.pid_wait();
    chassis.pid_turn_set(181, 90);
    chassis.pid_wait();
    setIntake(0);

    
    set_drive(-84 - 1, 110); // move to mogo
    chassis.pid_wait_until(-10);
    std::cout << "POSITION: " << chassis.odom_x_get() << " " << chassis.odom_y_get() << " " << chassis.odom_theta_get() << std::endl;
    double wallDist = (rightAlignmentSensor.get_distance() * 0.0393701) * abs(cos(chassis.odom_theta_get() * M_PI / 180));
    std::cout << "WALL DIST: " << wallDist << std::endl;
    if (leftAlignmentSensor.get_confidence() > 10) {
        chassis.odom_x_set(-72 + wallDist);
    }
    std::cout << "NEW POSITION: " << chassis.odom_x_get() << " " << chassis.odom_y_get() << " " << chassis.odom_theta_get() << std::endl;
    chassis.pid_wait_until(-45 - 5);
    //chassis.pid_odom_set({{-48, 20 + 2}, rev, 127});
    //chassis.pid_wait_until_point({-48, -10});
    chassis.pid_speed_max_set(55);
    chassis.pid_wait_until(-81);
    mogoClamp.toggle(); // clamp mogo
    chassis.pid_wait();

    

    /////////////////////////// SECOND MOGO ///////////////////////////
    /////////////////////////// SECOND MOGO ///////////////////////////

    chassis.pid_turn_set(90, 127); // turn to pure pursuit two stacks
    chassis.pid_wait();
    setIntake(127);
    callLBReset(); // reset ladybrown

    chassis.pid_drive_set(20 + 1, 110); // move to first ring
    chassis.pid_wait();
    chassis.pid_turn_set(63.5 + 1, 90); // turn to second ring
    chassis.pid_wait();
    chassis.pid_drive_set(78 + 1, 110); // move to second and third ring
    chassis.pid_wait_until(35);
    chassis.pid_speed_max_set(45);
    chassis.pid_wait_until(71);
    intake.move(100);
    ChangeLBState(PROPPED);
    chassis.pid_wait();
    chassis.pid_turn_set(70, 90); // turn to go back
    chassis.pid_wait();
    chassis.pid_drive_set(-46.5 + 0.5, 110); // move back a bit
    chassis.pid_wait();
    setIntake(0);
    chassis.pid_turn_set(0, 90); // turn to wall stake
    chassis.pid_wait();
    ChangeLBState(SEMIEXTENDED);

    ColorLoopActive = true;
    colorFiltrationActive = false;
    startColorUntil(1);
    chassis.pid_drive_set(17 + 10, 70, false, false);
    chassis.pid_wait_until(5);
    chassis.pid_speed_max_set(55);
    intake.move(100); // intake slower to stop inside intake

    chassis.pid_wait_until(17 - 1.5);
    //chassis.pid_wait();
    ChangeLBState(EXTENDED); // extend ladybrown
    //chassis.pid_wait();
    chassis.drive_set(127, 127);
    pros::delay(450);
    set_drive(-15, 700); // move back
    chassis.pid_wait_until(-11);
    ChangeLBState(PROPPED); // retract ladybrown for 2nd extension
    chassis.pid_wait();
    pros::delay(200 - 100);
    chassis.pid_drive_set(17, 70, false, false); // move to wall stake
    stopColorUntilFunction();
    chassis.pid_wait_until(4 - 2);
    chassis.pid_speed_max_set(55);
    setIntake(127); // intake into ladybrown
    chassis.pid_wait_until(13);
    setIntake(0);
    ChangeLBState(EXTENDED);
    chassis.pid_wait();
    setDrive(110, 110);
    long startMillis = pros::millis();
    while (pros::millis() - startMillis < 400 - 150) {
        chassis.drive_set(110, 110);
        pros::delay(10);
    }
    //pros::delay(400 - 150);

    chassis.pid_turn_set(0, 90); // turn to align
    chassis.pid_wait();

    set_drive(-13.5 - 1, 3000); // move back
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown
    setIntake(127);
    chassis.pid_turn_set(-90 + 1, 127); // turn to three rings
    chassis.pid_wait();
    std::cout << "POSITION: " << chassis.odom_x_get() << " " << chassis.odom_y_get() << " " << chassis.odom_theta_get() << std::endl;



    wallDist = (rightAlignmentSensor.get_distance() * 0.0393701) * abs(sin(chassis.odom_theta_get() * M_PI / 180));
    std::cout << "WALL DISTANCE: " << wallDist << std::endl;
    if (rightAlignmentSensor.get_confidence() > 10) {
        chassis.odom_y_set(72 - wallDist - 5);
    }

    intake.move_voltage(12000);
    stopColorUntilFunction();

    // collecting 3 rings

    set_drive(57, 2000, 0, 70);
    //chassis.pid_odom_set({{-55.13, 48}, fwd, 90});
    callLBReset();
    chassis.pid_wait_until(29 - 3);
    //chassis.cancelMotion();
    set_drive(28.3 + 3 + 1, 2500, 0, 55); // intake rings slowly
    chassis.pid_wait();
    pros::delay(200 - 100);
    std::cout << "POSITION: " << chassis.odom_x_get() << " " << chassis.odom_y_get() << " " << chassis.odom_theta_get() << std::endl;
    chassis.pid_turn_set(40, 70); // turn to last ring before corner
    chassis.pid_wait_quick_chain();
    set_drive(13.5 - 2, 1500, 75, 120); // move to ring before corner
    chassis.pid_wait_quick_chain();
    //pros::delay(100);
    chassis.pid_turn_set(112 + 5, 100 - 45); // turn to corner
    chassis.pid_wait_quick_chain();

    set_drive(-13 - 3, 1000, 0, 60); // back into corner
    chassis.pid_wait();
    callLBReset();

    //intake.move(-127);
    //pros::delay(300 - 100);
    intake.move(0);
    mogoClamp.toggle(); // unclamp mogo

    ColorLoopActive = true;

    // TRANSITION PERIOD
    // TRANSITION PERIOD

    chassis.pid_turn_set(138 - 1.5, 90); // turn to intake ring
    chassis.pid_wait();
    startColorUntil(1); // stop first red ring at top
    intake.move(0);
    set_drive(83 - 1, 3000, 80, 127); // go to intake ring
    chassis.pid_wait_until(40);
    chassis.pid_speed_max_set(80);
    intake.move(0);
    chassis.pid_wait_until(62 + 5);
    //chassis.pid_speed_max_set(70);
    intake.move(105);
    chassis.pid_wait();
    // set_drive(-3,3000,80,110);
    // chassis.pid_wait();
    chassis.pid_turn_set(45, 90); // turn to second ring
    chassis.pid_wait();
    set_drive(31.5 - 3, 2000, 0, 75); // go to second ring
    chassis.pid_wait_until(4);
    stopColorUntilFunction();
    intake.move(0);
    chassis.pid_wait_until(25+2);
    ChangeLBState(PROPPED);
    stopColorUntilFunction();
    //chassis.pid_wait_until(29);
    chassis.pid_wait();
    intake.move_voltage(12000);
    pros::delay(350);
    setIntake(0);
    ChangeLBState(SEMIEXTENDED);
    pros::delay(150 - 50);
    startColorUntil(1);
    intake.move(110);
    chassis.pid_drive_set(6 + 3, 110); // move in a little more
    chassis.pid_wait();
    // chassis.pid_drive_set(-5, 110); // move back
    // chassis.pid_wait();

    chassis.pid_turn_set(-45 + 7 - 2, 90); // turn to mogo
    chassis.pid_wait();
    chassis.pid_drive_set(-34.5 - 0.5, 2000); // move to mogo
    //chassis.pid_odom_set({{44.13 - 4, 9 + 3}, rev, 110});
    chassis.pid_wait_until(-14);
    chassis.pid_speed_max_set(70);
    chassis.pid_wait_until(-30);
    mogoClamp.toggle();
    std::cout << "POSITION: " << chassis.odom_x_get() << " " << chassis.odom_y_get() << " " << chassis.odom_theta_get() << std::endl;


    /////////////////////////// THIRD MOGO ///////////////////////////
    /////////////////////////// THIRD MOGO ///////////////////////////

    chassis.pid_wait();
    stopColorUntilFunction();
    chassis.pid_turn_set(90, 90); // turn to AWS
    chassis.pid_wait();
    intake.move(127);
    chassis.pid_drive_set(15, 70, false, false); // move to AWS
    chassis.pid_wait_until(5);
    chassis.pid_speed_max_set(50);
    chassis.pid_wait();

    //reset at wall stake
    double x_pos = 72 - (8.5) * sin(chassis.odom_theta_get() * M_PI / 180.0);
    double y_pos = -cos(chassis.odom_theta_get() * M_PI / 180.0) * 8;

    chassis.odom_xy_set(x_pos, y_pos);

    chassis.pid_turn_set(90, 90); // turn to wall stake
    chassis.pid_wait();
    ChangeLBState(FULLEXTENDED); // extend ladybrown
    pros::delay(45 + 10);
    chassis.pid_drive_set(-16, 1500, false, false); // move back
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown
    chassis.pid_turn_set(-135, 90); // turn to get ring outside of ladder
    chassis.pid_wait();
    //set_drive(28 + 2 + 3); // move to ring outside ladder
    chassis.pid_odom_set({{23.2 + 1, -24.13 + 1}, fwd, 110});
    chassis.pid_wait();
    std::cout << "AFTER AWS POSITION: " << chassis.odom_x_get() << " " << chassis.odom_y_get() << " " << chassis.odom_theta_get() << std::endl;
    
    chassis.pid_turn_set(136 - 5, 60); // turn to intake first two stack 
    chassis.pid_wait();
    intake.move_voltage(12000);
    colorFiltrationActive = true;
    set_drive(29 - 2); // go to intake first two stack
    chassis.pid_wait();
    chassis.pid_turn_set(100 - 5, 90 - 15); // turn to intake second two stack
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(13 - 2, 110); // go to intake second two stack
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(80, 90); // turn to go back
    chassis.pid_wait_quick_chain();
    //startColorUntil(1);
    //chassis.pid_turn_set(225, 90 - 10); // turn to intake third two stack
    //chassis.pid_wait();
    //stopColorUntilFunction();
    intake.move(127);
    chassis.pid_drive_set(-35, 110); // go back
    chassis.pid_wait();
    chassis.pid_turn_set(107 - 4, 110); // turn to intake third two stack
    chassis.pid_wait_quick_chain();
    rightDoinker.toggle();
    //startColorUntil(1);
    set_drive(30 - 1); // go to intake third two stack
    chassis.pid_wait_quick_chain();
    //chassis.pid_turn_set(105, 90); // turn to corner
    //chassis.pid_wait();
    //rightDoinker.toggle();
    //set_drive(12);
    //chassis.pid_wait();/
    chassis.pid_turn_behavior_set(ez::ccw);
    chassis.pid_turn_set(-75 + 30, 110); // turn to corner
    chassis.pid_wait();
    chassis.pid_turn_behavior_set(ez::shortest);
    intake.move(0);
    // long startTime = pros::millis();
    // while (pros::millis() - startTime < 700) {
        //     chassis.drive_set(-120, -120);
        //     pros::delay(20);
        // }
    set_drive(-12 + 3); // back into to corner
    chassis.pid_wait_until(-1);
    mogoClamp.toggle(); // unclamp mogo
    chassis.pid_wait();


    rightDoinker.toggle();
    set_drive(15 + 1.5);
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(15 + 1.5, 110); // turn to last mogo
    chassis.pid_wait_quick_chain();
    intake.move(127);

    /////////////////////////// FOURTH MOGO ///////////////////////////
    /////////////////////////// FOURTH MOGO ///////////////////////////

    chassis.slew_drive_constants_set(1_in, 127);

    chassis.pid_drive_set(1000, 127, true, true);
    chassis.pid_wait_until(80);
    setDrive(127 - 20, 127 - 20);
    long start = pros::millis();
    while (pros::millis() - start < 800) { // manual timeout of 1 second
        pros::delay(20);
    }
    intake.move(-127);
    ChangeLBState(EXTENDED);
    // chassis.pid_turn_set(44, 120); // turn to hang on ladder
    // chassis.pid_wait_quick_chain();
    // chassis.pid_drive_set(-68 - 4, 127, false);
    // chassis.pid_wait_until(-40 - 5);
    // chassis.pid_speed_max_set(70)
    intake.move(0);
    intakeUnstuckActivated = false;
    colorFiltrationActive = false;
    chassis.pid_turn_set(44 + 10, 120); // turn to hang on ladder
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-20 + 4, 127, false);
    chassis.pid_wait_quick_chain();
    intake.move(0);
    chassis.pid_turn_set(44, 120); // turn to hang on ladder
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-50 - 4, 127, false);
    intake.move(0);
    chassis.pid_wait_until(-24 - 4);
    chassis.pid_speed_max_set(80);
    intake.move(0);
    chassis.pid_wait_quick_chain();


/*
    chassis.pid_wait();
    pros::delay(200);
    //ChangeLBState(EXTENDED); // extend ladybrown a little
    setIntake(127);
    chassis.pid_turn_set(90 - 2, 127); // turn to AWS
    chassis.pid_wait();
    set_drive(4.5 + 10, 1500 - 500, 0, 70); // move to AWS
    chassis.pid_wait();
    set_drive(-8, 1500); // move back
    setIntake(0);
    ChangeLBState(FULLEXTENDED); // extend ladybrown
    pros::delay(700);
    set_drive(-5 + 0.25, 2000, 60, 120); // move back
    setIntake(127);

    chassis.pid_turn_set(-45, 127); // turn to pure pursuit 3 rings
    chassis.pid_wait();
    //chassis.follow(skills5_txt, 13, 3500); // pure pursuit 3 rings top right
    chassis.pid_wait();
    set_drive(-16, 1000, 70, 127); // move back after done
    chassis.pid_wait();
    chassis.pid_turn_set(180, 127); // turn to bottom left rings
    chassis.pid_wait();
    set_drive(140, 2500, 120, 127); // go to bottom left rings
    chassis.pid_wait();

    chassis.pid_turn_set(90, 127); // turn to intake second stack
    chassis.pid_wait();
    set_drive(10, 1500, 60, 120); // intake second stack
    chassis.pid_wait();
    //pros::delay(500);
    chassis.pid_turn_set(-145 - 5, 127); // turn to third two stack
    chassis.pid_wait();
    set_drive(13, 2000); // intake third two stack
    chassis.pid_wait();
    rightDoinker.toggle();
    pros::delay(500);

    
    chassis.pid_turn_set(-58 + 5, 127); // turn to corner, sweep with rightDoinker
    //chassis.swingToHeading(-55 + 20, lemlib::DriveSide::LEFT, 1000); // turn to corner
    chassis.pid_wait();
    //pros::delay(400);
    mogoClamp.toggle(); // unclamp mogo
    set_drive(-6 - 5, 1200, 100, 120); // move to corner
    chassis.pid_wait();
    setIntake(0);
    setIntake(0);
    //pros::delay(200);
    chassis.pid_turn_set(-45, 127); // turn to corner
    chassis.pid_wait();
    set_drive(20 - 3, 2000, 90, 127); // move out of corner
    chassis.pid_wait();
    chassis.pid_turn_set(20, 127); // turn to 4th mogo


    /////////////////////////// FOURTH MOGO ///////////////////////////
    /////////////////////////// FOURTH MOGO ///////////////////////////


    chassis.pid_wait();
    callLBReset();
    set_drive(108 - 4, 3200, 120, 127); // push other mogo to corner
    chassis.pid_wait();
    set_drive(-10, 1500, 0, 60); // move back

}

*/
}

void skillsMacro() {
    colorFiltrationActive = false;
    chassis.odom_xyt_set(-60.5, -13, (-49 + 3)); // starts at middle of red alliance line
    //pros::Task lb_task(LBLoop);
    LBState = EXTENDED;
    ladybrown2.set_zero_position(-46);
    ChangeLBState(FULLEXTENDED);
    intakeUnstuckActivated = true;
    ColorLoopActive = true;

    pros::delay(200);
    intake.move(-127);
    pros::delay(400);
    intake.move(0);

    set_drive(-19.5, 1500, 0, 70); // move away from alliance stake
    chassis.pid_wait_until(-16);

    mogoClamp.toggle(); // clamp mogo
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown
    master.rumble("."); // short rumble to notify driver

}

void recalibrateUsingDistance() {
    // double theta = chassis.odom_theta_get() % 360.0;
    // if (theta < 0) {
    //     theta += 360.0;
    // }
    // if (theta > 270) {

    // }
}