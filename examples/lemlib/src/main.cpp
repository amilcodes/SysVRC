// Robot setup for the LemLib example. auton_import.py reads the Drivetrain,
// ControllerSettings, OdomSensors and drive curves from here.
#include "main.h"
#include "lemlib/api.hpp"

// six-motor blue drive, 3.25" omnis at 450 rpm
pros::MotorGroup leftMotors({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({4, 5, 6}, pros::MotorGearset::blue);
pros::Imu imu(10);

// a vertical and a horizontal tracking wheel on rotation sensors
pros::Rotation verticalEnc(11);
pros::Rotation horizontalEnc(-12);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.75);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -3.5);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.5, lemlib::Omniwheel::NEW_325, 450, 2);

lemlib::ControllerSettings lateralController(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 0);

lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors, &throttleCurve, &steerCurve);

pros::Motor intake(7, pros::MotorGearset::blue);
pros::adi::DigitalOut clamp('A');

void initialize() { chassis.calibrate(); }

void autonomous() { soloAwp(true); }

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    while (true) {
        chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        pros::delay(10);
    }
}
