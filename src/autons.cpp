#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
const int SWING_SPEED = 90;

// These are out of 127
using namespace ez;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(16.0, 0.0, 100.0);         // ez::fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(9.5, 0.0, 20);        // Holds the robot straight while going forward without odom
  // WITH MOGO: chassis.pid_turn_constants_set(4.2 + 0.5, -0.05, 30, 15.0);     // Turn in place constants, old are 3.05, 0.05, 25.5, 15, negative kI are 2.5, -0.1, 10, 15
  chassis.pid_turn_constants_set(3.7, 0, 20, 15.0);     // Turn in place constants, old are 3.05, 0.05, 25.5, 15, negative kI are 2.5, -0.1, 10, 15
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(100_ms, 3_deg, 250_ms, 7_deg, 150_ms, 500_ms); // velocity original is 500 ms
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 200_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 1_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.odom_reset();

  chassis.pid_drive_set(24_in, DRIVE_SPEED, false, true);
  chassis.pid_wait();
  /*for (int i=0;i<4;i++) {
    chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
    chassis.pid_wait();
  }*/

  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at
  // chassis.pid_drive_set(24_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(180, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(24_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_wait();
  chassis.pid_turn_set(45_deg, 110);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_turn_set(135_deg, 110);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_turn_set(270_deg, 110);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait();
  
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 127);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, 127);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, 127);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, 127);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, 127);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, 127);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

// ORIENT YOURSELF TO THE FIELD:
// The field is oriented with the red alliance on the left and the blue alliance on the right.
// The center mobile goal should be lower.
// This orientation makes a cartesian plane with the origin at the center of the field.

void set_drive(double inches, int time, float minSpeed, float maxSpeed) {

    ez::pose c_pose = chassis.odom_pose_get();
    double trueAngle = c_pose.theta;
    chassis.pid_drive_set(inches, maxSpeed);
}

using namespace std;

int sign(bool isBlue) {
    return isBlue?1:-1;
}

/*
  goal: move as fast as possible in a straight line, ignoring accuracy
 */
void moveMax(double dist, int timeout) {
  // //these two variables used to track distance traveled
  // lemlib::Pose lastPose = chassis.getPose();
  // lemlib::Pose curPose = chassis.getPose();
  // double distTravelled = 0;
  // long startMillis = pros::millis(); // used to see how much time has elapsed
  // while (distTravelled < fabs(dist) && pros::millis() - startMillis < timeout) { // if robot has traveled far enough or time has gone over the limit
  //   chassis.tank(127 * sgn(dist), 127 * sgn(dist)); // move at max speed, account for backwards
  //   curPose = chassis.getPose();
  //   distTravelled += curPose.distance(lastPose); // find distance between positions and add to distance traveled
  //   lastPose = chassis.getPose();
  //   pros::delay(10);
  // }
  // setDrive(0, 0); // stop moving
}

void simpleRing(bool isBlue) {
	int sgn=isBlue?1:-1;
	chassis.odom_xyt_set(0, 0, 90 * sgn);
	set_drive(-34 - 2, 2500, 0, 65);
	chassis.pid_wait_until(28);
	mogoClamp.toggle();
	chassis.pid_wait();
	//pros::delay(1000);
	pros::delay(500);
	intake.move_voltage(12000);
	pros::delay(700);
	chassis.pid_turn_set(0, 127);
	chassis.pid_wait();
	set_drive(21, 2000);
	chassis.pid_wait();
	pros::delay(1500);
	set_drive(-10, 1500);
	chassis.pid_wait();
	chassis.pid_turn_set(180, 127);
	chassis.pid_wait();
	set_drive(30, 2500, 0, 70);
	chassis.pid_wait();
  pros::delay(1000);
  intake.move(0);
	
}

void verySimpleMogo(bool isBlue) {
	int sgn=isBlue?1:-1;
	chassis.odom_xyt_set(0, 0, 90 * sgn);
	set_drive(-34 - 2, 2500, 0, 65);
	chassis.pid_wait_until(28);
	mogoClamp.toggle();
	chassis.pid_wait();
	//pros::delay(1000);
	pros::delay(500);
	intake.move_voltage(12000);
	pros::delay(700);
	chassis.pid_turn_set(180, 127);
	chassis.pid_wait();
	set_drive(21, 2000);
	chassis.pid_wait();
	pros::delay(1500);
	set_drive(-10, 1500);
	chassis.pid_wait();
	chassis.pid_turn_set(0, 127);
	chassis.pid_wait();
	set_drive(30, 2500, 0, 70);
	chassis.pid_wait();
  pros::delay(1000);
  intake.move(0);
	
}

void simpleMogo(bool isBlue) {
	int sgn=isBlue?1:-1; // variable to reverse autonomous
	chassis.odom_xyt_set(0, 0, (33 + 1) * sgn);

  // Set up ladybrown position
  LBState = PROPPED;
  ladybrown2.set_zero_position(-46); // ladybrown currently 46 degrees above
  ChangeLBState(FULLEXTENDED);
  pros::delay(650); // delay until ladybrown finishes scoring
  set_drive(-15, 2000);
  chassis.pid_wait();
  ChangeLBState(REST);
  chassis.pid_turn_set(90 * sgn, 127); // turn to mobile goal
  chassis.pid_wait();
  set_drive(-23 - 11 + 3, 2000, 0, 70);
  chassis.pid_wait_until(25 + 1);
  mogoClamp.toggle(); // clamp mobile goal
  chassis.pid_wait();
	intake.move_voltage(12000);
	chassis.pid_turn_set(180, 127); // turn to two stack ring
	chassis.pid_wait();
	set_drive(21, 2000); // score first ring
	chassis.pid_wait();
	pros::delay(500);
	set_drive(-20, 1500);
	chassis.pid_wait();
  chassis.pid_turn_set((45 - 2) * sgn, 127); // turn to middle two stack
	chassis.pid_wait();
	set_drive(30, 2000, 0, 60); // move to color sort wrong ring
	chassis.pid_wait();
  intake.move(127);
  pros::delay(1000);
  set_drive(15, 1500); // intake correct ring
  chassis.pid_wait();
  set_drive(-24 - 15, 1500, 70, 120);
  chassis.pid_wait();
  chassis.pid_turn_set(-45 * sgn, 127); // turn to ladder
  chassis.pid_wait();
  ChangeLBState(EXTENDED); // touch ladder
	
}

void newMogoRush(bool isBlue) {

 // PRELOAD near corner, set up to intake two stack during first movement

 colorFiltrationActive = false;

int sgn=isBlue?1:-1;
 //chassis.odom_xyt_set(0, 0, (-63+180) * sgn); // Set position
 LBState = PROPPED;
 ladybrown2.set_zero_position(-25); // ladybrown currently 46 degrees above
 ChangeLBState(SEMIEXTENDED);
 chassis.odom_xyt_set(0, 0, (-113) * sgn); // Set position
 set_drive(33.5 + 0.5, 2500, 126, 127); // Move to first mogo
 pros::delay(50);
 intake.move(-127);
 startColorUntil(1);
 //leftDoinker.toggle();
 pros::delay(200);
 intake.move(127);
 chassis.pid_wait_until(12);
 if (isBlue) {
  rightDoinker.toggle();
 } else {
  leftDoinker.toggle();
 }
 chassis.pid_wait_until(30 + 1);
 if (isBlue) {
  rightDoinker.toggle();
 } else {
  leftDoinker.toggle();
 }
 chassis.pid_wait_until(31.5 + 1);
 // pros::delay(500);
 // chassis.pid_turn_set(-150 * sgn); // Turn mogo to disrupt
 // chassis.pid_wait();
 set_drive(-13 - 3, 1500, 120); // Move back
 chassis.pid_wait();
 if (isBlue) {
  rightDoinker.toggle();
 } else {
  leftDoinker.toggle();
 }
 pros::delay(300);
 set_drive(-7);
 chassis.pid_wait();
 if (isBlue) {
  rightDoinker.toggle();
 } else {
  leftDoinker.toggle();
 }
 std::cout <<"CUR THETA: " << chassis.odom_theta_get() << std::endl;
 
 chassis.pid_turn_set((74 + 5) * sgn, 90); // Turn to first mogo
 chassis.pid_wait();
 set_drive(-22, 2000, 0, 90); // Move to first mogo
 chassis.pid_wait_until(-10);
 chassis.pid_speed_max_set(65);
 chassis.pid_wait_until(-20 + 2);
 mogoClamp.toggle(); // Clamp first mogo
 chassis.pid_wait();
 stopColorUntilFunction();
 intake.move(127);
 chassis.pid_turn_set((66 + 7) * sgn, 120); // Turn to second mogo
 chassis.pid_wait();
 set_drive(32 + 3.5, 2000, 65, 127); // Move to second mogo
 //chassis.pid_wait_until(15);
 //mogoClamp.toggle();
 chassis.pid_wait();
 
 chassis.pid_turn_set((-135) * sgn, 90); // Turn to drop mogo
 chassis.pid_wait();
 mogoClamp.toggle(); // Drop mogo
 intake.move(127);
 chassis.pid_drive_set(9.5, 127); // move out from first mogo
 chassis.pid_wait();
 colorFiltrationActive = true;
 chassis.pid_turn_set((132) * sgn, 90); // Turn to second mogo
 chassis.pid_wait();
 set_drive(-19, 2000, 0, 80); // Move to second mogo
 chassis.pid_wait_until(-10 + 5);
 chassis.pid_speed_max_set(65);
 chassis.pid_wait_until(-17 + 3);
 mogoClamp.toggle(); // Clamp second mogo
 chassis.pid_wait();
 chassis.pid_turn_set((130 + 50) * sgn, 127); // turn to go back a bit
 chassis.pid_wait();
 // set_drive(-11 - 3);
 // chassis.pid_wait_until(-9 - 3);
 // mogoClamp.toggle();
 // chassis.pid_wait();
 //allianceColorBlue = !allianceColorBlue;
 //startColorUntil(1);
 intake.move(0);
 set_drive(35 - 10);
  chassis.pid_wait_until(10);
  //chassis.pid_speed_max_set(60);
 chassis.pid_wait_quick_chain();
chassis.pid_turn_set((120) * sgn, 100); // Turn to drop mogo
chassis.pid_wait_quick_chain();
//   intake.move(-127);
//   chassis.pid_turn_set(135, 127); // turn to corner
//   chassis.pid_wait_quick_chain();
//   intake.move(127);

 //chassis.pid_targets_reset();
 //setDrive(75, 175);
 long startMillis = pros::millis();
 while (pros::millis() - startMillis < 400) {
   chassis.drive_set(100, 100);
   pros::delay(10);
 }
 startMillis = pros::millis();
 while (pros::millis() - startMillis < 700) {
   chassis.drive_set(75, 75);
   pros::delay(10);
 }
 //setDrive(0, 0);
 //pros::delay(200);
 chassis.pid_drive_set(-10, 127);
 chassis.pid_wait();
 //intake.move(-127);
 chassis.pid_turn_set(256 * sgn, 127);
 chassis.pid_wait();
 set_drive(40 + 0.5);
 //allianceColorBlue = !allianceColorBlue;
 chassis.pid_wait();
 chassis.pid_turn_set((-135 + 3) * sgn, 90);
 chassis.pid_wait();
 ChangeLBState(EXTENDED);
 set_drive(4);
 chassis.pid_wait();
}

void disruptRingRush(bool isBlue) {
  int sgn=isBlue?1:-1;
	chassis.odom_xyt_set(55 * sgn,30,-(71 + 2.5) * sgn); //Starting Line 71
  setIntake(0);
  set_drive(38 + 1.5, 2500, 100, 127); //move and grab to rings 
  leftDoinker.toggle();
  rightDoinker.toggle();
  chassis.pid_wait();
  set_drive(-12 + 1.5 - 3, 2000); // move back a little
  chassis.pid_wait();
  chassis.pid_turn_set(-97 * sgn, 127); // turn to go back
  chassis.pid_wait();
  set_drive(-4.5 - 3, 1500, 70, 127); // move back
  chassis.pid_wait_until(1 + 3);
  leftDoinker.toggle();
  rightDoinker.toggle();
  chassis.pid_wait();
  // chassis.turnToPoint(24 * sgn, 21, 3000, {.forwards = false}); // turn to mogo
  //chassis.swingToHeading(55 * sgn, isBlue?lemlib::DriveSide::LEFT:lemlib::DriveSide::RIGHT, 3000);
  chassis.pid_turn_set((33 - 2) * sgn, 127); // turn to mogo
  chassis.pid_wait();
  set_drive(-18.5 - 5 + 3, 1500, 0, 65 + 10);
  chassis.pid_wait_until(14 + 1.5);
  mogoClamp.toggle(); // clamp mogo
  set_drive(-12 + 1.5, 1500, 70, 120); // move to mogo
  chassis.pid_wait();
  chassis.pid_turn_set((-8) * sgn, 127); // turn to intake rings
  //chassis.turnToPoint((24 - 4) * sgn, 60, 3000); // turn to intake rings
  chassis.pid_wait();
  pros::delay(100);
  set_drive(41 - 15, 2000, 0, 50 - 5); // move to intake rings
  chassis.pid_wait_until(8);
  intake.move(127); // start intake
  chassis.pid_wait();
  pros::delay(500);
  set_drive(13 - 2, 1500);
  chassis.pid_wait();
  
  pros::delay(500);
  chassis.pid_turn_set((135 - 1.5) * sgn, 127); // turn to preload
  chassis.pid_wait();
  set_drive(58.5 - 10, 2000, 70, 127); // move to preload
  chassis.pid_wait_until(40 - 30);
  ChangeLBState(PROPPED);
  chassis.pid_wait();
  pros::delay(900-300);
  chassis.pid_turn_set((125 - 5.5) * sgn, 127);
  chassis.pid_wait();
  intake.move(0);
  set_drive(11 - 2.25, 1500); // move to preload and score
  chassis.pid_wait();
  ChangeLBState(3);
  pros::delay(700 - 100);
  set_drive(-24.5 + 3 + 8, 2000, 70, 127); // move back
  chassis.pid_wait();
  ChangeLBState(REST);
  
}

void disruptRingRushBlue() {
  int sgn= 1;
	chassis.odom_xyt_set(55 * sgn,30,-(71 + 2.5) * sgn); //Starting Line 71
  setIntake(-127);
  set_drive(38 + 0.25, 2500, 100, 127); //move and grab to rings 
  //leftDoinker.toggle();
  rightDoinker.toggle();
  chassis.pid_wait();
  intake.move(0);
  set_drive(-12 + 1.5 - 3, 2000); // move back a little
  chassis.pid_wait();
  chassis.pid_turn_set(-97 * sgn, 127); // turn to go back
  chassis.pid_wait();
  set_drive(-4.5 - 3, 1500, 70, 127); // move back
  chassis.pid_wait_until(1 + 3);
  leftDoinker.toggle();
  rightDoinker.toggle();
  chassis.pid_wait();
  // chassis.turnToPoint(24 * sgn, 21, 3000, {.forwards = false}); // turn to mogo
  //chassis.swingToHeading(55 * sgn, isBlue?lemlib::DriveSide::LEFT:lemlib::DriveSide::RIGHT, 3000);
  chassis.pid_turn_set((33 - 2) * sgn, 127); // turn to mogo
  chassis.pid_wait();
  set_drive(-18.5 - 5 + 3, 1500, 0, 65 + 10);
  chassis.pid_wait_until(14 + 1.5);
  mogoClamp.toggle(); // clamp mogo
  set_drive(-12 + 1.5, 1500, 70, 120); // move to mogo
  chassis.pid_wait();
  chassis.pid_turn_set((-8) * sgn, 127); // turn to intake rings
  //chassis.turnToPoint((24 - 4) * sgn, 60, 3000); // turn to intake rings
  chassis.pid_wait();
  pros::delay(100);
  set_drive(41 - 15, 2000, 0, 50 - 5); // move to intake rings
  chassis.pid_wait_until(8);
  intake.move(127); // start intake
  chassis.pid_wait();
  pros::delay(500);
  set_drive(13 - 2, 1500);
  chassis.pid_wait();
  
  pros::delay(500);
  chassis.pid_turn_set((135 - 1.5) * sgn, 127); // turn to preload
  chassis.pid_wait();
  set_drive(58.5 - 10, 2000, 70, 127); // move to preload
  chassis.pid_wait_until(40 - 30);
  ChangeLBState(PROPPED);
  chassis.pid_wait();
  pros::delay(900-300);
  chassis.pid_turn_set((125 - 5.5) * sgn, 127);
  chassis.pid_wait();
  intake.move(0);
  set_drive(11 - 2.25, 1500); // move to preload
  chassis.pid_wait();
  ChangeLBState(3);
  pros::delay(700 - 100);
  set_drive(-24.5 + 3 + 8, 2000, 70, 127); // move back
  chassis.pid_wait();
  ChangeLBState(REST);
  chassis.pid_turn_set((-110) * sgn, 127); // turn to ladder
  chassis.pid_wait();
  set_drive(26.5, 1500, 90, 100); // move forward
  chassis.pid_wait();
  ChangeLBState(PROPPED); 
}



void safeRingSide(bool isBlue) {
	int sgn=isBlue?1:-1;
  intakeUnstuckActivated = true;
	chassis.odom_xyt_set(0, 0, (146) * sgn);
  //LBState = PROPPED;
  //LBRotation.set_position(4600);
// LB on aws
  //ladybrown2.set_zero_position(-46);
  //ChangeLBState(FULLEXTENDED);
  //pros::delay(250);
  intake.move(-127);
  pros::delay(400 - 100);
  intake.move(127);
  set_drive(-15, 2000, 90);
  chassis.pid_wait();
  ChangeLBState(REST);

// Wanna try to get the 2 stack early bc we start next to it

// Get 2 stack next to aws
  intake.move(90); // start intake
  //if (isBlue) {
  //  chassis.pid_turn_set(-(158 + 10)*sgn, 120);
  //} else {
    //chassis.pid_turn_set(-(158 + 5)*sgn, 120);
  //}
  
  //chassis.pid_wait();
  //intakeLift.toggle(); // lift intake
  //set_drive(10 + 10); // move into ring
  //chassis.pid_wait();
  //intakeLift.toggle(); // lower intake on ring
  startColorUntil(1);
  //set_drive(10,3000, 0, 70);
  //intake.move(127);
  //intake.move(90); // stop intake to keep ring on intake
  //chassis.pid_wait();
  //pros::delay(400-100);
  //set_drive(-15);
  //chassis.pid_wait();
  //set_drive(10);
  //chassis.pid_wait();
  //pros::delay(200 + 200);
  //if (isBlue) {
  //  set_drive(-31 + 4); // move away from ring
  //} else {
    //set_drive(-21 + 5, 2000, 0, 70); // move away from ring
  //}
  //set_drive(-31); // move away from ring
  
  //chassis.pid_wait();


// Getting mogo
  chassis.pid_turn_set(90 * sgn, 90);
  chassis.pid_wait();
  set_drive(-31 + .5, 2000, 0, 60);
  chassis.pid_wait_until(-27 - 1);
  mogoClamp.toggle();  
  //intakeUnstuckActivated=true;
  chassis.pid_wait();
  
  // Getting middle 2 stacks
  // if (isBlue) {
  //   chassis.pid_turn_set((-45 + 5) * sgn, 90); // Turn to first 2 stack
  // } else {
    chassis.pid_turn_set((-45) * sgn, 90); // Turn to first 2 stack
  //}
  //chassis.pid_turn_set((-45) * sgn, 90); // Turn to first 2 stack
  chassis.pid_wait();
  stopColorUntilFunction();
  // Intake ring
  intake.move(127);
  intake.move(127);
  //if (isBlue) {
  //  set_drive(21 - 4.5 + 2); // Move to first 2 stack
  //} else {
    set_drive(16.5 + 1.5); // Move to first 2 stack
  //}
  
  chassis.pid_wait();
  // Turn to last 2 stack
  chassis.pid_turn_set((-5) * sgn, 100);
  chassis.pid_wait();
  intake.move(127);
  set_drive(12 + 10); // Intake 2 stack
  chassis.pid_wait();
  // chassis.pid_turn_relative_set(-5*sgn,127);
  // chassis.pid_wait();
  //pros::delay(200-100);
  // if (isBlue) {
  //   set_drive(15 - 5.5); // forward
  // } else {
  //   set_drive(14+1); // forward
  // }
  //set_drive(14+1); // forward
  chassis.pid_wait();
  pros::delay(500-150);
  if (isBlue) {
    set_drive(-28 + 9); // back
  } else {
    set_drive(-28); // back
  }
  setIntake(0);
  chassis.pid_wait();
  if (isBlue) {
    chassis.pid_turn_set((70-25 + 10) * sgn, 127); // Turn to final 2 stack on our quarter
  } else {
    chassis.pid_turn_set((70-25) * sgn, 127); // Turn to final 2 stack on our quarter
  }
  //chassis.pid_turn_set((70-25) * sgn, 127); // Turn to final 2 stack on our quarter
  chassis.pid_wait();
  set_drive(25,3000,127);
  chassis.pid_wait_until(5);
  intake.move(127);
  chassis.pid_wait();
  //chassis.pid_turn_relative_set(-20*sgn,127);
  //    1chassis.pid_wait_quick_chain();
  //intakeUnstuckActivated=false;
  //set_drive(-41,3000,127);
  //ChangeLBState(SEMIEXTENDED);
  chassis.pid_turn_set(70 * sgn, 100);
  chassis.pid_wait();
  set_drive(40, 2000, 127);
  
  chassis.pid_wait();/*
  chassis.pid_turn_set(65 * sgn, 90); // Turn to corner
  chassis.pid_wait();
  set_drive(45); // drive into corner
  chassis.pid_wait();
  chassis.pid_turn_set(45 * sgn, 90);
  chassis.pid_wait();
  set_drive(-10); // Move out of corner a bit
  chassis.pid_wait();
*/
  /*
  chassis.pid_turn_set(180 * sgn, 90);
  chassis.pid_wait();
  intakeLift.toggle();
  set_drive(100);
  chassis.pid_wait_until(60);
  intakeLift.toggle();
  chassis.pid_wait();
  //Why do u guys need to do intake that ring like that
*/	
}

void MogoSide(bool isBlue) {
	chassis.odom_xyt_set(50_in,-36_in,114.3_deg); // Touching Red Ring Facing Mogo
	chassis.pid_drive_set(-(19.5 - 1), 127); // Move to Mogo
  chassis.pid_wait_until(17);
  mogoClamp.toggle(); // Clamp Mogo
  chassis.pid_wait();
	setIntake(127); // Score Ring
	chassis.pid_wait();
  chassis.pid_turn_set(307.75, 127); // Turn to Face Rings
  chassis.pid_wait();
  chassis.pid_drive_set((29.5 - 1), 127); // Reach Rings
  chassis.pid_wait_until(15);
  (isBlue?leftDoinker:rightDoinker).toggle(); // Toggle Doinker Rush Mech
  chassis.pid_wait();
  chassis.pid_turn_set(337.75_deg, 127); // Turn to Other Ring
  chassis.pid_wait();
  (!isBlue?leftDoinker:rightDoinker).toggle(); // Toggle Second Doinker Rush Mech
  chassis.pid_drive_set(-30, 127); // Move Back
  chassis.pid_wait();
  chassis.pid_turn_relative_set(30_deg, 127); // Turn To Ring
  chassis.pid_wait();
  chassis.pid_swing_set((!isBlue?ez::RIGHT_SWING:ez::LEFT_SWING), 180_deg, 90, 45); // Swing to Ring
  chassis.pid_wait();
  chassis.pid_turn_set({24,-48}, fwd, 127); // Turn to Ring 
  chassis.pid_wait();
  chassis.pid_drive_set(ez::util::distance_to_point({chassis.odom_x_get(),chassis.odom_y_get()},{24,-48}) - 1, 127); // Move to Ring
  chassis.pid_wait();
  chassis.pid_turn_set({66,66},fwd,127); // Turn to Corner
  chassis.pid_wait();
  chassis.pid_drive_set(ez::util::distance_to_point({chassis.odom_x_get(),chassis.odom_y_get()},{66,-66}) - 1, 127); // Move to Corner
  ChangeLBState(PROPPED); // Prop Lady Brown
  chassis.pid_wait();
  pros::delay(500);
  setIntake(0);
  chassis.pid_drive_set(-20,127); // Move Back
  chassis.pid_wait();
  chassis.pid_turn_set({6,-66},fwd,127); // Turn to Wall Stake
  chassis.pid_wait();
  chassis.pid_drive_set(ez::util::distance_to_point({chassis.odom_x_get(),chassis.odom_y_get()},{6,-66}) - 2, 127); // Move to Wall Stake
  chassis.pid_wait_quick();
  chassis.pid_turn_set({0,-72},fwd,127); // Correct to Wall Stake
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2,127); // Correct to Wall Stake
  chassis.pid_wait_quick();
  ChangeLBAuton(EXTENDED); // Score Ring on Wall Stake
  pros::delay(200);
  ChangeLBAuton(REST); // Bring LB Back
}

void RingRush6(bool isBlue) {
  if (isBlue) {
    chassis.odom_x_flip();
    chassis.odom_theta_flip();
  }
  int sgn = isBlue?1:-1;
  LBState=REST;
  chassis.odom_xyt_set(-48.5, 30, -68.4 * sgn);
  intake.move(-127);
  chassis.pid_drive_set(43 + 2,127); // Drive to Middle Ring
  (isBlue?rightDoinker:leftDoinker).toggle(); // Grab Ring
  chassis.pid_wait_until(20);
  intake.move(127);
  startColorUntil(1);
  chassis.pid_wait_until(42.5 - 1);
  //pros::delay(100);
  chassis.pid_drive_set(-19,127); // Move Back
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90,127);
  chassis.pid_wait();
  chassis.pid_drive_set(-8,127); // drive back more
  chassis.pid_wait();
  (isBlue?rightDoinker:leftDoinker).toggle(); // Release Ring
  pros::delay(100);
  chassis.pid_turn_set(-55,90);
  chassis.pid_wait();
  chassis.pid_drive_set(-20,127);
  chassis.pid_wait_until(-18);
  mogoClamp.toggle(); // Grab Mogo
  chassis.pid_wait();
  stopColorUntilFunction();
  intake.move(127);
  chassis.pid_turn_set(-12,127);
  chassis.pid_wait();
  intake.move(127);
  intake.move(127);
  chassis.pid_drive_set(30 - 3,127);
  chassis.pid_wait_quick_chain();
  //pros::delay(300 - 100);
  chassis.pid_turn_set(-75,100);
  chassis.pid_wait();
  intake.move(127);
  chassis.pid_drive_set(40,90); // Go To Corner
  //setDrive(90, 90);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_turn_set(-45,127); 
  chassis.pid_wait();
  chassis.pid_drive_set(-15 - 2,127); // move back from corner
  chassis.pid_wait();
  chassis.pid_turn_set(180,100); // turn to intake two rings
  chassis.pid_wait();
  chassis.pid_drive_set(46 - 2, 90); // move to intake two rings
  chassis.pid_wait_until(29);
  intakeLift.toggle();
  chassis.pid_wait();
  //pros::delay(200);
  leftDoinker.toggle();
  chassis.pid_turn_set(-90,127);
  ChangeLBState(PROPPED);
  chassis.pid_wait_until(10);
  intakeLift.toggle();
  chassis.pid_wait();
  leftDoinker.toggle();
  chassis.pid_drive_set(10, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8,127);
  intake.move(0);
  ChangeLBState(FULLEXTENDED);
  chassis.pid_wait();
  ChangeLBState(REST);
  chassis.pid_turn_set(90,127);
  chassis.pid_wait();
  chassis.pid_drive_set(50,127);
  chassis.pid_wait_until(10);
  chassis.pid_targets_reset();
}



void stateSoloAwp(bool isBlue) {


  // GETS CORNER

int sgn=isBlue?1:-1;
chassis.odom_xyt_set(0, 0, (180 - 34.18) * sgn);

// AWS

LBState = PROPPED; // Prop LB for preload
//LBRotation.set_position(4400);
ladybrown2.set_zero_position(-46 + 20);
ChangeLBState(EXTENDED); // Extend LB for AWS
pros::delay(200);
intake.move(-127);
pros::delay(300 - 50);

set_drive(-13.5 + 1.25, 2000, 120); // move back from AWS
chassis.pid_wait_until(-13 + 1.25);
ChangeLBState(REST); // retract ladybrown
chassis.pid_turn_set(90 * sgn, 90);
chassis.pid_wait();
callLBReset();
set_drive(-30.5 + 2, 2000, 120);
chassis.pid_wait_until(-10);
chassis.pid_speed_max_set(60);
chassis.pid_wait_until(-24 + 2);
mogoClamp.toggle(); // get mogo
chassis.pid_wait_until(-29.5 + 2);
intake.move(127);


intake.move_voltage(12000);
chassis.pid_turn_set((-49 + 4) * sgn, 90); // Turn to center line 2 stacks, first 2 stack there
chassis.pid_wait();
intake.move_voltage(12000);
set_drive(20 + 0.5, 1500, 110); // intake ring
chassis.pid_wait();
pros::delay(100);
chassis.pid_drive_set(-18.5 - 2, 127); // move back
chassis.pid_wait();

// Code for getting other ring in center 2 stacks
// chassis.pid_turn_set(10 + 1-0.5, 127);  // turns to other 2 stack in middle 2 stacks
// chassis.pid_wait();
// set_drive(20-6.5, 2000, 50, 90); // drive to other 2 stack
// chassis.pid_wait();
// set_drive(-13.5, 2000, 50, 90); // drive to other 2 stack
// chassis.pid_wait();

// Discarded swing turn mode for getting other ting in center 2 stacks
// chassis.pid_swing_set(ez::RIGHT_SWING, 30_deg, 127);
// chassis.pid_turn_set(30, 127); 
// chassis.pid_wait();
// set_drive(-12, 2000, 50, 90); 
// chassis.pid_wait();
intake.move(127);
chassis.pid_turn_set(0 * sgn, 90); // turn to final 2 stack in this quarter
chassis.pid_wait();
set_drive(12 + 7, 3000);
chassis.pid_wait();

chassis.pid_turn_set(-270 * sgn, 127); 
chassis.pid_wait();
set_drive(24, 2000, 50, 120); 
chassis.pid_wait();

intake.move(127);
chassis.pid_turn_set(180, 110);
chassis.pid_wait();
mogoClamp.toggle(); // release mogo 
startColorUntil(1);
colorFiltrationActive = false;
intakeUnstuckActivated = false;
//intakeLift.toggle();
set_drive(67, 3000, 0, 100); // intake both rings of middle 2 stack
// chassis.pid_wait_until(35);
// chassis.pid_speed_max_set(70);
chassis.pid_wait_until(50);
//intakeLift.toggle();
//chassis.pid_speed_max_set(110);
chassis.pid_wait();
chassis.pid_turn_set(-270 * sgn, 115); // turn to other mogo
chassis.pid_wait();
set_drive(-24 + 1, 200, 0, 90); // drive to other mogo
intakeUnstuckActivated = true;
chassis.pid_wait_until(-10);
chassis.pid_speed_max_set(65);
chassis.pid_wait();
mogoClamp.toggle(); // clamp other mogo
stopColorUntilFunction();
colorFiltrationActive = true;
intake.move(127);
chassis.pid_turn_set(180, 70); // turn to final two stack
chassis.pid_wait_quick_chain();
stopColorUntilFunction();
intake.move(127);
set_drive(24, 3000); // intake final two stack
chassis.pid_wait();
set_drive(-26 + 5, 3000, 0, 127); // drive to ladder
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-45 * sgn, 127); // turn to ladder
ChangeLBState(EXTENDED); // touch ladder
chassis.pid_wait();
//set_drive(5);

}


void safeFourRing(bool isBlue) {


  // GETS CORNER
// Note: it does need to be this fast
colorFiltrationActive = true;
ColorLoopActive = true;

int sgn=isBlue?1:-1;
chassis.odom_xyt_set(0, 0, (34 - 3) * sgn);

// AWS

LBState = PROPPED; // Prop LB for preload
//LBRotation.set_position(4400);
ladybrown2.set_zero_position(-46 + 5);
ChangeLBState(EXTENDED); // Extend LB for AWS
pros::delay(200);
intake.move(-127);
pros::delay(200);
ChangeLBState(EXTENDED);

set_drive(-13.5 - 1, 2000, 80); // move back from AWS
chassis.pid_wait();
ChangeLBState(REST); // retract ladybrown
chassis.pid_turn_set(90 * sgn, 127);
chassis.pid_wait();
callLBReset();
set_drive(-30 + 2, 2000, 90);
chassis.pid_wait_until(-13 + 4);
chassis.pid_speed_max_set(65);
// set_drive(-10.5, 1500, 0, 45); // move slower
chassis.pid_wait_until(-25 + 1);
mogoClamp.toggle(); //clamp mogo
chassis.pid_wait();
intake.move_voltage(12000);
chassis.pid_turn_set(180, 127); // turn to two stack
chassis.pid_wait();
// set_drive(27, 2000); // intake ring
// chassis.pid_wait();
  intake.move_voltage(12000);
  //ChangeLBState(PROPPED);
  set_drive(20 + 1.5, 2000, 110); // intake ring
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-3, 127); // move back
chassis.pid_wait();
intake.move_voltage(12000);
// 	set_drive(7.5 + 1, 2000, 90); 
// chassis.pid_wait();
chassis.pid_turn_set((130 - 5) * sgn, 90 + 5); // turn to corner
chassis.pid_wait();
// // while (pros::millis() - startMillis < 500 - 150) {
  // //   chassis.drive_set(110, 110);
  // //   pros::delay(10);
  // // }
  chassis.pid_drive_set(40 + 4, 127, false, false); // move to corner
  chassis.pid_wait_quick_chain();
  intakeUnstuckActivated = false;
  long startMillis = pros::millis();
while (pros::millis() - startMillis < 1300) {
  chassis.drive_set(80 - 15, 80 - 15);
  pros::delay(10);
}
intakeUnstuckActivated = true;
//pros::delay(1000);
//set_drive(-15, 3000, 0, 30); // move out of corner
//chassis.pid_wait();
//pros::delay(200 - 100); // wait a bit
//set_drive(10); // go back into corner
//chassis.pid_wait();

chassis.pid_turn_set(115 * sgn, 90);
pros::delay(750);
chassis.pid_turn_set((130 - 5) * sgn, 90 + 5); // turn to corner
chassis.pid_wait();
pros::delay(750 - 500);

set_drive(-5); // move out of corner #2
chassis.pid_wait();
// pros::delay(500); // wait a bit #2
// set_drive(10); // intake ring #2
// chassis.pid_wait();
// pros::delay(1000);


chassis.pid_turn_set((-43) * sgn, 95); // turn to corner
chassis.pid_wait_quick_chain();

set_drive(-47 + 2.5 + 1); // move back
chassis.pid_wait_until(-44 + 2.5 + 1);
set_drive(3, 2000, 110); // move to ladder
// chassis.pid_wait_until(5);
ChangeLBState(EXTENDED); // touch ladder
intake.move(0);
chassis.pid_wait();



pros::delay(20000);

//UNUSED
//UNUSED


set_drive(-16.5 - 28.75 - 1); // move back
chassis.pid_wait_quick();
intakeUnstuckActivated = true;

chassis.pid_turn_set(isBlue?(35 + 8):-35, 90); // turn to middle stack
chassis.pid_wait();

set_drive(51 - 8); // move to middle stack
chassis.pid_wait_until(15);
intakeUnstuckActivated = false;

chassis.pid_speed_max_set(50);
intakeLift.toggle();
chassis.pid_wait_quick();
intakeLift.toggle();



pros::delay(300 - 50);
//set_drive(8 + 7, 1500, 0, 50); // move 
intakeUnstuckActivated = true;
//chassis.pid_wait();
set_drive(-20 - 5, 1500, 127); // move back
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-40 * sgn, 90); // turn to ladder
chassis.pid_wait_quick_chain();
set_drive(12, 2000, 110); // move to ladder
chassis.pid_wait_until(5);
ChangeLBState(EXTENDED); // touch ladder
intake.move(0);
chassis.pid_wait();

// chassis.pid_turn_set(-90 * sgn, 90); // turn to middle mogo
// chassis.pid_wait();
// mogoClamp.toggle(); // release mogo
// chassis.pid_turn_set(90 * sgn, 90); // turn to middle mogo
// chassis.pid_wait();
// set_drive(-20 - 5, 2000, 110); // move to middle mogo
// chassis.pid_wait();
// chassis.pid_turn_set(60 * sgn, 90); // turn to middle mogo
// chassis.pid_wait();


// chassis.pid_turn_set((-45 * sgn), 127);
// chassis.pid_wait();

// set_drive(25, 2000, 120); // move to ladder
// chassis.pid_wait_until(12);
// ChangeLBState(EXTENDED);

// 	set_drive(-27, 1500); // move back

//   chassis.pid_turn_set(100 * sgn, 100); // turn to corner
//   chassis.pid_wait();
//   set_drive(40); // move to corner
//   chassis.pid_wait();
//   pros::delay(500);
//   set_drive(-10); // move back
//   chassis.pid_wait();
//   set_drive(10); // intake 2nd ring
//   chassis.pid_wait();
//   set_drive(-10); // move back for real
//   chassis.pid_wait();

//   chassis.pid_turn_set(0 * sgn, 127); // turn to middle two stack
//   chassis.pid_wait();

//   set_drive(51, 2000, 50); // move to middle two stack
//   intakeLift.toggle();
//   chassis.pid_wait();
//   intakeLift.toggle();
// pros::delay(500);

// 	set_drive(-27, 1500); // move back
// 	chassis.pid_wait();
//   chassis.pid_turn_set((-30) * sgn, 127);
//   chassis.pid_wait();

// set_drive(20, 2000, 120); // move to ladder
// chassis.pid_wait();
// ChangeLBState(EXTENDED);

}

void positiveSideQuals(bool isBlue) {
  double sgn = isBlue?1 : -1;
  chassis.odom_xyt_set(0, 0, -45 * sgn);

  chassis.drive_angle_set(-51 * sgn);

  LBState = PROPPED;
  ladybrown2.set_zero_position(-46);
  ChangeLBState(EXTENDED);

  pros::delay(600);

// Get to and turn towards mogo
  set_drive(-13, 110); // move backwards to about mogo
  chassis.pid_wait();
  chassis.pid_turn_set(0 * sgn, 80); // turn to mogo
  chassis.pid_wait();
  set_drive(-10); // move slightly more into the mogo


  ChangeLBState(REST);
  chassis.pid_wait();
  mogoClamp.toggle();
  set_drive(-30, 127);
  chassis.pid_wait_until(-28);
  mogoClamp.toggle();
  chassis.pid_wait();
  pros::delay(100);

  chassis.pid_turn_set(-42 * sgn, 100);
  pros::delay(100);
  chassis.pid_wait();

  set_drive(20, 127);
  pros::delay(300);
  intake.move(0);
  chassis.pid_wait_until(16);

  isBlue ? rightDoinker.toggle() : leftDoinker.toggle();

  pros::delay(250);

  chassis.pid_turn_set(65 * sgn, 100);
  pros::delay(100);
  chassis.pid_wait();
  
  if (!isBlue) {
    set_drive(4, 127);
    chassis.pid_wait();
  } else {
    set_drive(4, 127);
    chassis.pid_wait();
  }

  isBlue ? leftDoinker.toggle() : rightDoinker.toggle();

  pros::delay(250);

  if (!isBlue) {
    chassis.pid_swing_set(ez::RIGHT_SWING,45 * sgn, -100, 0);
    pros::delay(100);
    chassis.pid_wait();
  } else {
    chassis.pid_swing_set(ez::LEFT_SWING,45 * sgn, -100, 0);
    pros::delay(100);
    chassis.pid_wait();
  }

  set_drive(-45, 127);
  pros::delay(100);
  chassis.pid_wait_until(-43);

  // idk if need to be used, delete if it makes it to the commit
  //leftDoinker.set_value(false);
  //rightDoinker.set_value(false);

  pros::delay(250);

  chassis.pid_turn_set(-10 * sgn, 100);
  pros::delay(100);
  chassis.pid_wait();

  intake.move_velocity(600);

  if (!isBlue) {
    chassis.pid_swing_set(ez::RIGHT_SWING,200 * sgn, 100, 20);
    pros::delay(100);
    chassis.pid_wait();
  } else {
    chassis.pid_swing_set(ez::LEFT_SWING,200 * sgn, 100, 20);
    pros::delay(100);
    chassis.pid_wait();
  }

  set_drive(30, 127);
  pros::delay(100);
  chassis.pid_wait_until(28);

  chassis.pid_turn_set(260.5 * sgn, 100);
  pros::delay(100);
  chassis.pid_wait();

  set_drive(30.5, 127);
  pros::delay(100);
  chassis.pid_wait_until(7);
  isBlue ? leftDoinker.toggle()  : rightDoinker.toggle();
  chassis.pid_wait_until(28.5);

  intake.move(0);

  chassis.pid_turn_set(340 * sgn, 100);
  pros::delay(200);
  chassis.pid_wait();

  mogoClamp.toggle();

  set_drive(15, 127);
  pros::delay(100);
  chassis.pid_wait_until(10);

  leftDoinker.toggle();
  leftDoinker.toggle();

  chassis.pid_turn_set(405 * sgn, 100);
  pros::delay(100);
  chassis.pid_wait_until(403 * sgn);

  set_drive(54, 127);
  pros::delay(100);
  chassis.pid_wait_until(54);
}





void stateSoloAwpCenterGet(bool isBlue) {

  // Quick chains need tuning
  // GETS CORNER

int sgn=isBlue?1:-1;
chassis.odom_xyt_set(0, 0, (180 - 34 + .3) * sgn);

// AWS

LBState = PROPPED; // Prop LB for preload
//LBRotation.set_position(4400);
ladybrown2.set_zero_position(-46 + 20);
ChangeLBState(EXTENDED); // Extend LB for AWS
pros::delay(200);
intake.move(-127);
pros::delay(200);

set_drive(-11 -1-1.5, 2000, 80); // move back from AWS
chassis.pid_wait();
ChangeLBState(REST); // retract ladybrown
chassis.pid_turn_set(90 * sgn, 90);
chassis.pid_wait();
callLBReset();
set_drive(-30.5 + 2, 2000, 90);
chassis.pid_wait_until(-15);
chassis.pid_speed_max_set(-60);
chassis.pid_wait_until(-24);
mogoClamp.toggle(); // get mogo
chassis.pid_wait_until(-29.5 + 2);
intake.move(127);


intake.move_voltage(12000);
chassis.pid_turn_set(-52 * sgn, 90); // Turn to center line 2 stacks, first 2 stack there
chassis.pid_wait_quick_chain();
intake.move_voltage(12000);
set_drive(20 + 2, 1500, 110); // intake ring
chassis.pid_wait_quick();
//pros::delay(300);

// Code for getting other ring in center 2 stacks
// chassis.pid_turn_set(10 + 1-0.5, 127);  // turns to other 2 stack in middle 2 stacks
// chassis.pid_wait();
// set_drive(20-6.5, 2000, 50, 90); // drive to other 2 stack
// chassis.pid_wait();
// set_drive(-13.5, 2000, 50, 90); // drive away from other 2 stack
// chassis.pid_wait();

// Discarded swing turn mode for getting other ting in center 2 stacks
// chassis.pid_swing_set(ez::RIGHT_SWING, 30_deg, 127);
// chassis.pid_turn_set(30, 127); 
// chassis.pid_wait();
// set_drive(-12, 2000, 50, 90); 
// chassis.pid_wait();
intake.move(127);
chassis.pid_turn_set(-315 * sgn, 90); // turn to final 2 stack in this quarter
chassis.pid_wait();
set_drive(20-8, 3000);
chassis.pid_wait();

chassis.pid_turn_set(270, 127); 
chassis.pid_wait_quick();
set_drive(24 + 4, 2000, 50, 90); 
chassis.pid_wait_quick_chain();




// Get corner here
intake.move(127);
chassis.pid_turn_set(45*sgn, 90); // Turn to corner
chassis.pid_wait_quick_chain();
set_drive(24 + 3, 127); // Drive into corner
chassis.pid_wait_quick_chain();
pros::delay(200);
set_drive(-24, 110); // Drive out of corner
chassis.pid_wait();


chassis.pid_turn_set(180 + 4, 110);
chassis.pid_wait_quick();

mogoClamp.toggle(); // release mogo 
startColorUntil(2);
set_drive(66 +1, 3000, 0, 85); // intake both rings of middle 2 stack
chassis.pid_wait();
chassis.pid_turn_set(270, 115); // turn to other mogo
chassis.pid_wait_quick_chain();
set_drive(-24 + 1, 200, 0, 90); // drive to other mogo
chassis.pid_wait();
mogoClamp.toggle(); // clamp other mogo
stopColorUntilFunction();
intake.move(127);
chassis.pid_turn_set(180, 70); // turn to final two stack
chassis.pid_wait_quick_chain();
stopColorUntilFunction();
intake.move(127);
set_drive(24, 3000); // intake final two stack
chassis.pid_wait_quick();
set_drive(-26 + 5, 3000, 0, 127); // drive to ladder
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-30 * sgn, 90); // turn to ladder
chassis.pid_wait();
leftDoinker.toggle();
rightDoinker.toggle();
set_drive(20);
chassis.pid_wait_quick_chain();
set_drive(-30);


}

void ringWalk(bool isBlue) {
  
  // Quick chains need tuning
  // GETS CORNER

int sgn=isBlue?1:-1;
chassis.odom_xyt_set(0, 0, (180 - 35) * sgn);
//pros::delay(800);
colorFiltrationActive = false;

// AWS

LBState = PROPPED; // Prop LB for preload
//LBRotation.set_position(4400);
ladybrown2.set_zero_position(-27);
ChangeLBState(FULLEXTENDED); // Extend LB for AWS
pros::delay(200);
intake.move(-127);
pros::delay(275 - 50);

set_drive(-11 -1-1.5, 2000, 120); // move back from AWS
chassis.pid_wait();
ChangeLBState(REST); // retract ladybrown
chassis.pid_turn_set(90 * sgn, 90);
chassis.pid_wait();
callLBReset();
set_drive(-30.5 + 2, 2000, 80);
chassis.pid_wait_until(-12 + 3);
chassis.pid_speed_max_set(60);
chassis.pid_wait_until(-24);
mogoClamp.toggle(); // get mogo
intake.move_voltage(12000);
chassis.pid_wait_until(-29.5 + 2);


ChangeLBState(REST);
chassis.pid_turn_set((-46 + 0.5) * sgn, 90); // Turn to center line 2 stacks, first 2 stack there
chassis.pid_wait();
intake.move_voltage(12000);
intake.move_voltage(12000);
stopColorUntilFunction();
set_drive(18 - 0.25, 1500, 110); // intake ring
colorFiltrationActive = true;
chassis.pid_wait_quick();
// pros::delay(300);

// Code for getting other ring in center 2 stacks
chassis.pid_turn_set((-14 + 4 - 5) * sgn, 127);  // turns to other 2 stack in middle 2 stacks
chassis.pid_wait();
set_drive(15.5 - 2.5, 2000, 50, 120); // drive to other 2 stack
chassis.pid_wait();
chassis.pid_turn_set(-15 * sgn, 120); // turn to other 2 stack
chassis.pid_wait();
set_drive(-17 + 2.5, 2000, 50, 120); // drive away from other 2 stack
chassis.pid_wait();

// Discarded swing turn mode for getting other ting in center 2 stacks
// chassis.pid_swing_set(ez::RIGHT_SWING, 30_deg, 127);
// chassis.pid_turn_set(30, 127); 
// chassis.pid_wait();
// set_drive(-12, 2000, 50, 90); 
// chassis.pid_wait();
intake.move(0);


chassis.pid_turn_set(50 * sgn, 127); 
chassis.pid_wait_quick();
//startColorUntil(1);
set_drive(15 + 1, 2000, 50, 120); // intake 4th two stakc
chassis.pid_wait_until(2);
intake.move(127);
chassis.pid_wait_quick_chain();
//pros::delay(300 - 150);
chassis.pid_drive_set(-5 - 2, 127);
chassis.pid_wait_quick_chain();
chassis.pid_turn_chain_constant_set(2);
chassis.pid_drive_chain_constant_set(2);
chassis.pid_turn_set(90 * sgn, 127); // turn to corner
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(18 + 9, 127); // drive to corner
chassis.pid_wait_quick_chain();
intake.move(127);
chassis.pid_turn_set((54 - 11) * sgn, 127); // turn to corner
chassis.pid_wait_quick_chain();
// set_drive(20 + 4, 2000, 50, 120); 
// chassis.pid_wait();
// //pros::delay(500);

// // Get corner here
// intake.move(127);
// chassis.pid_turn_set((60 + 10)*sgn, 90); // Turn to corner
// chassis.pid_wait_quick_chain();
// set_drive(27 - 10, 127); // Drive into corner
intake.move(127);
long start = pros::millis();
while (pros::millis() - start < 1500) {
  chassis.drive_set(80 + 20, 80 + 20);
  pros::delay(20);
}
intake.move(127);
//chassis.pid_wait_quick_chain();
//pros::delay(200);
chassis.pid_turn_set(45 * sgn, 127); // Turn to go to ladder
pros::delay(500);
intake.move(127);
pros::delay(500);
//chassis.pid_drive_set(-15, 127); // Drive out of corner
//chassis.pid_wait_quick_chain();
//chassis.pid_drive_set(10, 127); // Drive into corner
//chassis.pid_wait_quick_chain();
//pros::delay(500);
set_drive(-53.5, 75); // Drive out of corner
chassis.pid_wait();


chassis.pid_turn_set(-135 * sgn, 110);
pros::delay(250);
ChangeLBState(EXTENDED);
chassis.pid_wait_quick();

// chassis.pid_turn_set(-176 * sgn, 110);
// chassis.pid_wait_quick();

// startColorUntil(1);
// set_drive(66 +1, 3000, 0, 85); // intake both rings of middle 2 stack
// chassis.pid_wait();
// chassis.pid_turn_set(-45 * sgn,127);
// chassis.pid_wait();
// chassis.pid_drive_set(-30,127);
// chassis.pid_wait();
}

void barcbotsMogoRush(bool isBlue) {

  // PRELOAD near corner, set up to intake two stack during first movement

  int sgn=isBlue?1:-1;
  //chassis.odom_xyt_set(0, 0, (-63+180) * sgn); // Set position
  LBState = PROPPED;
  ladybrown2.set_zero_position(-25); // ladybrown currently 46 degrees above
  ChangeLBState(SEMIEXTENDED);
  chassis.odom_xyt_set(0, 0, (-113) * sgn); // Set position


  ChangeLBState(SEMIEXTENDED); // Move LB out of way
  intake.move(127);
  set_drive(33 + 1.5, 2500, 127, 127); // Move to first mogo


  startColorUntil(1); // color sort the rings
  //pros::delay(200);
  intake.move(127); // intake color sorted rings
  rightDoinker.toggle(); // toggle right doinker to push red ring towards bot


  chassis.pid_wait_until(12);
  leftDoinker.toggle(); // doinker down to get mogo

  chassis.pid_wait_until(28.5 + 1.5);
  leftDoinker.toggle(); // Lifts doinker up to mogo so it holds the mogo

  chassis.pid_wait_until(33 - 1.5 + 1.5);

// Uncomment to turn this into a disrupt
  // pros::delay(500);
  // chassis.pid_turn_set(-150 * sgn); // Turn mogo to disrupt
  // chassis.pid_wait();

  set_drive(-18 - 1.5, 1500, 120); // Move back with mogo
  
  chassis.pid_wait();
  leftDoinker.toggle(); // releases mogo from doinker
  pros::delay(300);
  set_drive(-7, 1200, 111, 127); // go backwards to go to other mogo
  chassis.pid_wait_until(-3);
  rightDoinker.toggle();
  chassis.pid_wait_until(-5);
  intake.move(0);
  chassis.pid_wait();
  leftDoinker.toggle(); // puts doinker back up
  std::cout <<"CUR THETA: " << chassis.odom_theta_get() << std::endl;
  

  chassis.pid_turn_set((135 -  5) * sgn, 90); // Turn to first mogo
  chassis.pid_wait();
  set_drive(-22, 2000); // Move to first mogo
  chassis.pid_wait_until(-22 + 2);
  mogoClamp.toggle(); // Clamp first mogo
  intake.move(127);
  chassis.pid_wait();
  pros::delay(200);
  mogoClamp.toggle();
  stopColorUntilFunction();

  // chassis.pid_turn_set(-135 * sgn, 90);
  // chassis.pid_wait();
  // mogoClamp.toggle();
  // chassis.pid_drive_set(7, 110);
  // chassis.pid_wait();

  chassis.pid_turn_set((-20 - 15) * sgn, 90); // Turn to second mogo
  chassis.pid_wait();
  set_drive(-30 - 3, 2000, 65, 127); // Move to second mogo
  chassis.pid_wait_until(-20);
  mogoClamp.toggle();
  chassis.pid_wait();
  
  
  chassis.pid_turn_set((120) * sgn, 90); // turn to corner
  chassis.pid_wait();
  // set_drive(-11 - 3);
  // chassis.pid_wait_until(-9 - 3);
  // mogoClamp.toggle();
  // chassis.pid_wait();
  set_drive(30);
  chassis.pid_wait_quick_chain();
  //chassis.pid_targets_reset();
  setDrive(127, 127);
  long startMillis = pros::millis();
  while (pros::millis() - startMillis < 800 - 100) {
    chassis.drive_set(127, 127);
    pros::delay(10);
  }
  //setDrive(0, 0);
  //pros::delay(200);
  chassis.pid_drive_set(-10, 127); // Drive out of corner
  chassis.pid_wait_until(-8);
  chassis.pid_turn_set(256 * sgn, 127); // Turn to drive towards wall stake
  chassis.pid_wait();
  set_drive(42); // Drive to wall stake
  chassis.pid_wait();
  chassis.pid_turn_set((-135 - 5) * sgn, 90); // Turn to wall stake
  chassis.pid_wait();
  ChangeLBState(EXTENDED); // Get wall stake
  set_drive(2);
  chassis.pid_wait();
}

// void worldsMogoRush(bool isBlue) {

//   // PRELOAD near corner, set up to intake two stack during first movement
 
//   colorFiltrationActive = false;
 
//  int sgn=isBlue?1:-1;
//   //chassis.odom_xyt_set(0, 0, (-63+180) * sgn); // Set position
//   LBState = PROPPED;
//   ladybrown2.set_zero_position(-25); // ladybrown currently 46 degrees above
//   ChangeLBState(SEMIEXTENDED);
//   chassis.odom_xyt_set(0, 0, (-89) * sgn); // Set position
//   set_drive(32 + 4, 2500, 126, 127); // Move to first mogo
//   pros::delay(100);
//   intake.move(-127);
//   //startColorUntil(1);
//   //leftDoinker.toggle();
//   //pros::delay(200);
//   //intake.move(127);
//   chassis.pid_wait_until(12 - 6);
//   if (isBlue) {
//    rightDoinker.toggle();
//   } else {
//    leftDoinker.toggle();
//   }
//   pros::delay(150);
//   intake.move(0);
//   chassis.pid_wait_until(30.5 - 0.75);
//   if (isBlue) {
//    rightDoinker.toggle();
//   } else {
//    leftDoinker.toggle();
//   }
//   chassis.pid_wait_until(30.5 + 4);
//   // pros::delay(500);
//   // chassis.pid_turn_set(-150 * sgn); // Turn mogo to disrupt
//   // chassis.pid_wait();
//   set_drive(-8 - 4, 1500, 120); // Move back
//   chassis.pid_wait();
//   if (isBlue) {
//    rightDoinker.toggle();
//   } else {
//    leftDoinker.toggle();
//   }
//   pros::delay(300);
//   set_drive(-7 - 5);
//   chassis.pid_wait();
//   if (isBlue) {
//    rightDoinker.toggle();
//   } else {
//    leftDoinker.toggle();
//   }
//   std::cout <<"CUR THETA: " << chassis.odom_theta_get() << std::endl;
  
//   chassis.pid_turn_set((-57) * sgn, 90); // Turn to first ring
//   chassis.pid_wait();
//   intake.move(127);
//   set_drive(17 - 1+2); // Move to first ring
//   chassis.pid_wait_until(16);
//   set_drive(-2,127); // Move back a bit to intake ring
//   chassis.pid_wait_until(-1);
//   chassis.pid_turn_set((90 + 3) * sgn, 90); // Turn to first mogo)
//   //intake.move(100);
//   //pros::delay(100);
//   //intake.move(80);
//   intake.move(0);
//   //chassis.pid_wait_until(2);
//   chassis.pid_wait();
//   set_drive(-17 + 5-0.5-2, 2000, 0, 80); // Move to first mogo
//   chassis.pid_wait_until(-7);
//   chassis.pid_speed_max_set(70);
//   chassis.pid_wait_until(-10 + 2);
//   mogoClamp.toggle(); // Clamp first mogo
//   intake.move(127);
//   //stopColorUntilFunction();
//   chassis.pid_wait_until(-11);
//   chassis.pid_turn_set(90 * sgn, 90); // Turn to second mogo
//   chassis.pid_wait();
//   chassis.pid_drive_set(21 + 2, 127); // Move forward
//   chassis.pid_wait_until(19 + 2);
//   chassis.pid_turn_set(0 * sgn, 90); // Turn against wall
//   intake.move(-127);
//   chassis.pid_wait_quick_chain();
//   mogoClamp.toggle(); // drop first mogo
//   chassis.pid_turn_set(135 * sgn, 90); // Turn to second mogo
//   intake.move(-127);
//   chassis.pid_wait();
//   chassis.pid_drive_set(-27, 100); // Move to second mogo
//   chassis.pid_wait_until(-10);
//   chassis.pid_speed_max_set(65);
//   chassis.pid_wait_until(-25);
//   mogoClamp.toggle(); // Clamp second mogo
//   chassis.pid_wait();

//   //chassis.pid_drive_set(43 - 20, 127); // Move to push ring
//   //intake.move(127);
//   //chassis.pid_wait_until(15);
//   //chassis.pid_wait_until(25 - 3);
//   //mogoClamp.toggle(); // Release mogo
//   // chassis.pid_wait();
//   // //chassis.pid_drive_set(-5, 127); // Move back a bit
//   // //chassis.pid_wait();
//   // chassis.pid_turn_set((135 - 7.5 + 1) * sgn, 90); // Turn to second mogo
//   // chassis.pid_wait();
//   // set_drive(-35 - 3-2, 2000, 65, 90); // Move to second mogo
//   // chassis.pid_wait_until(-10);
//   // chassis.pid_speed_max_set(60);
//   // chassis.pid_wait_until(-30 - 3);
//   // mogoClamp.toggle(); // Clamp second mogo
//   // chassis.pid_wait();
//   // intake.move(127);
//   // //startColorUntil(1);
//   chassis.pid_drive_set(15, 127); // Move to push ring
//   chassis.pid_wait_quick_chain();
//   // chassis.pid_turn_set((135.5 - 2 + 1) * sgn, 90); // Turn to corner
//   // chassis.pid_wait_quick_chain();
//   // intake.move(127);
//   // set_drive(30+1);
//   // chassis.pid_wait_quick_chain();
//   chassis.pid_turn_set(130 * sgn, 90); // Turn to corner
//   chassis.pid_wait_quick_chain();
//   intake.move(127);
//   intakeUnstuckActivated =false;
//   long startMillis = pros::millis();
//   while (pros::millis() - startMillis < 1000 + 500) {
//    chassis.drive_set(80, 80);
//    pros::delay(10);
//   }

//   chassis.pid_drive_set(-39, 127);
//   chassis.pid_wait_until(-37);
//   chassis.pid_turn_set(220,127);
//   chassis.pid_wait();
//   chassis.pid_drive_set(40-5 - 1,127);
//   chassis.pid_wait();
//   ChangeLBState(EXTENDED);/*
//   intake.move(100);
//  chassis.pid_wait_until(-9);
//  intakeUnstuckActivated = true;
//  chassis.pid_turn_set(256 * sgn, 127);
//  chassis.pid_wait();
//  set_drive(40.5 - 1.5);
//  chassis.pid_wait_until(5);
//  stopColorUntilFunction();
//   intake.move(127);
//  //allianceColorBlue = !allianceColorBlue;
//  chassis.pid_wait();
//  chassis.pid_turn_set((-135 + 3) * sgn, 90);
//  chassis.pid_wait();
//  ChangeLBState(EXTENDED);
//  set_drive(4 - 1);
//  chassis.pid_wait();*/


// }

void worldsMogoRush(bool isBlue) {

  // PRELOAD near corner, set up to intake two stack during first movement
 
  colorFiltrationActive = false;
 
 int sgn=isBlue?1:-1;
  //chassis.odom_xyt_set(0, 0, (-63+180) * sgn); // Set position
  LBState = PROPPED;
  ladybrown2.set_zero_position(-25); // ladybrown currently 46 degrees above
  ChangeLBState(SEMIEXTENDED);
  chassis.odom_xyt_set(0, 0, (-89) * sgn); // Set position
  set_drive(32 + 4, 2500, 126, 127); // Move to first mogo
  pros::delay(100);
  intake.move(-127);
  startColorUntil(1);
  //leftDoinker.toggle();
  //pros::delay(200);
  //intake.move(127);
  chassis.pid_wait_until(12 - 6);
  if (isBlue) {
   rightDoinker.toggle();
  } else {
   leftDoinker.toggle();
  }
  pros::delay(150);
  intake.move(0);
  chassis.pid_wait_until(27 + 3.5);
  if (isBlue) {
   rightDoinker.toggle();
  } else {
   leftDoinker.toggle();
  }
  chassis.pid_wait_until(30.5 + 4);
  // pros::delay(500);
  // chassis.pid_turn_set(-150 * sgn); // Turn mogo to disrupt
  // chassis.pid_wait();
  set_drive(-12, 1500, 120); // Move back
  chassis.pid_wait_until(-9);
  if (isBlue) {
   rightDoinker.toggle();
  } else {
   leftDoinker.toggle();
  }
  chassis.pid_wait();
  pros::delay(300 - 50);
  set_drive(-7 - 5);
  chassis.pid_wait();
  if (isBlue) {
   rightDoinker.toggle();
  } else {
   leftDoinker.toggle();
  }
  std::cout <<"CUR THETA: " << chassis.odom_theta_get() << std::endl;
  
  chassis.pid_turn_set((-57) * sgn, 90); // Turn to first ring
  chassis.pid_wait();
  intake.move(127);
  set_drive(17 - 1+2); // Move to first ring
  chassis.pid_wait_until(16);
  intake.move(100);
  set_drive(-2,127); // Move back a bit to intake ring
  chassis.pid_wait();
  intake.move(85);
  chassis.pid_turn_set(90 * sgn, 90); // Turn to first mogo
  chassis.pid_wait_until(2);
  //intake.move(0);
  intake.move(45 + 10);
  chassis.pid_wait();
  set_drive(-17 + 5, 2000, 0, 80); // Move to first mogo
  chassis.pid_wait_until(-7);
  chassis.pid_speed_max_set(70);
  chassis.pid_wait_until(-10 + 2);
  mogoClamp.toggle(); // Clamp first mogo
  intake.move(127);
  stopColorUntilFunction();
  chassis.pid_wait_until(-11);
  chassis.pid_drive_chain_constant_set(2);
  chassis.pid_drive_set(43 - 5, 127); // Move to push ring
  intake.move(127);
  chassis.pid_wait_until(15 - 9);
  mogoClamp.toggle(); // Release mogo
  intake.move(-127);
  chassis.pid_wait_quick_chain();
  //chassis.pid_drive_set(-5, 127); // Move back a bit
  //chassis.pid_wait();
  chassis.pid_turn_set((127 - 2) * sgn, 90); // Turn to second mogo
  chassis.pid_wait();
  set_drive(-35 - 3, 2000, 65, 90); // Move to second mogo
  chassis.pid_wait_until(-10);
  chassis.pid_speed_max_set(60);
  chassis.pid_wait_until(-30 - 3);
  mogoClamp.toggle(); // Clamp second mogo
  chassis.pid_wait();
  intake.move(127);
  //startColorUntil(1);
  chassis.pid_turn_set((135.5 + 3) * sgn, 90); // Turn to corner
  chassis.pid_wait_quick_chain();
  intake.move(127);
  set_drive(30);
  chassis.pid_wait_quick_chain();
  intake.move(127);
  intakeUnstuckActivated =false;
  long startMillis = pros::millis();
  while (pros::millis() - startMillis < 1500 - 500) {
   chassis.drive_set(100, 100);
   pros::delay(10);
  }


  chassis.pid_drive_set(-10, 127);
  intake.move(127);
 chassis.pid_wait_until(-8);
 chassis.pid_drive_set(5, 127); // Drive out of corner
 chassis.pid_wait_quick_chain();
 chassis.pid_drive_set(-5, 127);
 chassis.pid_wait_quick_chain();
 intakeUnstuckActivated = true;
 chassis.pid_turn_set((256 - 4) * sgn, 127);
 chassis.pid_wait();
 set_drive(39);
 chassis.pid_wait_until(5);
 stopColorUntilFunction();
 mogoClamp.toggle();
  intake.move(127);
 //allianceColorBlue = !allianceColorBlue;
 chassis.pid_wait_until(38 - 1);
 chassis.pid_turn_set((-135 + 3) * sgn, 90);
 chassis.pid_wait();
 ChangeLBState(EXTENDED);
 set_drive(3);
 chassis.pid_wait();


}