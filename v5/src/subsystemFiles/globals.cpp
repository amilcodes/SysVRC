#include "main.h"
#include "pros/motors.h"
#include "lemlib/api.hpp"



//MOTORS

//drive
pros::MotorGroup left_side_motors({-8, 9, -10}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::MotorGroup right_side_motors({4, -1, 2}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
// pros::MotorGroup left_side_motors({5, -6, 10}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
// pros::MotorGroup right_side_motors({-2, 1, -7}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

//intake
pros::Motor intake(-16, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

// ladybrown wall stake mech
pros::Motor ladybrown1(14, pros::v5::MotorGears::rpm_200, pros::v5::MotorUnits::degrees);
pros::Motor ladybrown2(-17, pros::v5::MotorGears::rpm_200, pros::v5::MotorUnits::degrees);


//pistons
pros::adi::Pneumatics intakeLift('B', false);
pros::adi::Pneumatics mogoClamp('A', false);
pros::adi::Pneumatics leftDoinker('C', false);
pros::adi::Pneumatics rightDoinker('H', false);

// inertial
pros::Imu IMU(21);

bool allianceColorBlue = true;

// rotational sensor
pros::Rotation LBRotation(15);

pros::Optical optical(15);

pros::Distance autoClampSensor(1);
pros::Distance rightAlignmentSensor(11);
pros::Distance leftAlignmentSensor(1);


//CONTROLLERS
pros::Controller master(pros::E_CONTROLLER_MASTER);

    // Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {9, -8, -10},     // Left Chassis Ports (negative port will reverse it!)
    {-1, 4, 2},  // Right Chassis Ports (negative port will reverse it!)

    21,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450,   // Wheel RPM = cartridge * (motor gear / wheel gear)
    1.205);

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels
