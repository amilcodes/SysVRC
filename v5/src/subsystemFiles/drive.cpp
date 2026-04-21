#include "main.h"
#include "subsystemHeaders/intake.hpp"
#define DRIVE_DEADBAND 0.1f
#define DRIVE_SLEW 0.02f
#define CD_TURN_NONLINEARITY 0.65
       // This factor determines how fast the wheel
       // traverses the "non linear" sine curve
#define CD_NEG_INERTIA_SCALAR 4.0
#define CD_SENSITIVITY 1.00

// Helper Functions
void setDrive(int left, int right) {
    left_side_motors.move(left);
    right_side_motors.move(right); 
}

void setDriveVelocity(int left, int right) {
    left_side_motors.move_velocity(left);
    right_side_motors.move_velocity(right);
}

void setDriveVoltage(int left, int right) {
    left_side_motors.move_velocity(left);
    right_side_motors.move_velocity(right);
}

void resetDriveEncoders() {
    left_side_motors.tare_position();
    right_side_motors.tare_position();
}

void brakeModeCoast() {
    left_side_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_side_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void brakeModeHold() {
    left_side_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_side_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// Driver Control Functions
void setDriveMotors() {

    // Split 2 joystick arcade drive
    int leftJoystick = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightJoystick = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    int left = leftJoystick + rightJoystick;
    int right = leftJoystick - rightJoystick;

    setDrive(left, right);
}



void runArcadeDrive() {
    // Get the power and turn values from the joystick
	int turn = master.get_analog(ANALOG_LEFT_Y);
	int power = master.get_analog(ANALOG_RIGHT_X);
    // Calculate the left and right side powers
	int left = power + turn;
	int right = power - turn;
    // Set the drive motors
	setDrive(left, right);	
}

void runCheesyDrive() {
  double prevTurn = 0.0;
  double prevThrottle = 0.0;
  double quickStopAccumlator = 0.0;
  double negInertiaAccumlator = 0.0;
  while (true) {
		int throttle = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
    std::pair<double, double> output = cheesyDrive(throttle, turn, prevTurn, prevThrottle, quickStopAccumlator, negInertiaAccumlator);
		int left = (int) output.first;
		int right = (int) output.second;

        setDrive(left, right);
        
        pros::lcd::print(3, "quickstopaccumulator: %f", quickStopAccumlator);
        pros::lcd::print(4, "neginertiaaccumulator: %f", negInertiaAccumlator);
        pros::lcd::print(1, "original left: %f new: %d", throttle - turn, left);
        
        

        prevTurn = turn;
	    prevThrottle = throttle;

        setIntakeMotors();

		pros::delay(20);
	}
}

// We apply a sinusoidal curve (twice) to the joystick input to give finer
// control at small inputs.
static double _turnRemapping(double iturn) {
	double denominator = sin(M_PI / 2 * CD_TURN_NONLINEARITY);
	double firstRemapIteration =
	    sin(M_PI / 2 * CD_TURN_NONLINEARITY * iturn) / denominator;
	return sin(M_PI / 2 * CD_TURN_NONLINEARITY * firstRemapIteration) / denominator;
}

// On each iteration of the drive controller (where we aren't point turning) we
// constrain the accumulators to the range [-1, 1].

static void _updateAccumulators(double& quickStopAccumlator, double& negInertiaAccumlator) {
	if (negInertiaAccumlator > 1) {
		negInertiaAccumlator -= 1;
	} else if (negInertiaAccumlator < -1) {
		negInertiaAccumlator += 1;
	} else {
		negInertiaAccumlator = 0;
	}

	if (quickStopAccumlator > 1) {
		quickStopAccumlator -= 1;
	} else if (quickStopAccumlator < -1) {
		quickStopAccumlator += 1;
	} else {
		quickStopAccumlator = 0.0;
	}
}

std::pair<double, double> cheesyDrive(double ithrottle, double iturn, double prevTurn, double prevThrottle, double& quickStopAccumlator, double& negInertiaAccumlator) {
	bool turnInPlace = false;
	double linearCmd = ithrottle;
	if (fabs(ithrottle) < DRIVE_DEADBAND && fabs(iturn) > DRIVE_DEADBAND) {
		// The controller joysticks can output values near zero when they are
		// not actually pressed. In the case of small inputs like this, we
		// override the throttle value to 0.
		linearCmd = 0.0;
		turnInPlace = true;
	} else if (ithrottle - prevThrottle > DRIVE_SLEW) {
		linearCmd = prevThrottle + DRIVE_SLEW;
	} else if (ithrottle - prevThrottle < -(DRIVE_SLEW * 2)) {
		// We double the drive slew rate for the reverse direction to get
		// faster stopping.
		linearCmd = prevThrottle - (DRIVE_SLEW * 2);
	}

	double remappedTurn = _turnRemapping(iturn);

	double left, right;
	if (turnInPlace) {
		// The remappedTurn value is squared when turning in place. This
		// provides even more fine control over small speed values.
		left = remappedTurn * std::abs(remappedTurn);
		right = -remappedTurn * std::abs(remappedTurn);

	} else {
		double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
		negInertiaAccumlator += negInertiaPower;

		double angularCmd =
		    fabs(linearCmd) *  // the more linear vel, the faster we turn
		        (remappedTurn + negInertiaAccumlator) *
		        CD_SENSITIVITY -  // we can scale down the turning amount by a
		                          // constant
		    quickStopAccumlator;

		right = left = linearCmd;
		left += angularCmd;
		right -= angularCmd;

		_updateAccumulators(quickStopAccumlator, negInertiaAccumlator);
	}
	
	return std::make_pair(left, right);
}