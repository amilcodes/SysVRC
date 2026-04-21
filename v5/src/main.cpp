#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

long runStart;
double averageTempFahrenheit = 0;
double offset;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  pros::lcd::initialize();
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	LBRotation.reset_position();
  ladybrown2.tare_position();
	optical.set_led_pwm(100);
	allianceColorBlue = true; // VERY IMPORTANT
	initColorSort();
	std::cout << "initialize done" << "\n";
	initializeSelector();

  autoClampActivated = false;


  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Initialize chassis and auton selector
  chassis.initialize();
  offset = 72 - rightAlignmentSensor.get() * 0.0393701;

  std::cout << "offset: " << offset << "\n";
  //ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

void logger() {
    while (!pros::competition::is_disabled()) {
        //std::cout << "RED: " << std::to_string(optical.get_rgb().red) << " BLUE: " << std::to_string(optical.get_rgb().blue) << "\n";
		//std::cout << "DIFFERENCE: " << std::to_string(optical.get_rgb().blue - optical.get_rgb().red) << "\n";
		//std::cout << "HUE: " + to_string(optical.get_hue()) << "\n";
    auto pose = chassis.odom_pose_get();
    double wallDist = (rightAlignmentSensor.get_distance() * 0.0393701 + offset) * abs(cos(chassis.odom_theta_get() * M_PI / 180));
    //std::cout << "POSITION: (" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ", " + std::to_string(pose.theta) + ")\n";
		//std::cout << lemlib::format_as(chassis.getPose()) << "\n";
		//std::cout << ladybrown.get_actual_velocity() << "\n";
	  //std::cout << "Ladybrown Angle: " << ladybrown2.get_position() / 3.0 << " LBState: " << LBState << "\n";
		//std::cout << "intake voltage: " << intake.get_voltage() << "\n";
		//std::cout << "VELOCITY: " + std::to_string(intake.get_actual_velocity()) << " VOLTAGE: " + std::to_string(intake.get_voltage()) << "\n";
    //std::cout << "DISTANCE: " + std::to_string(rightAlignmentSensor.get_confidence()) << "\n";
		std::cout << "PROXIMITY: " << optical.get_proximity() << " DIFFERENCE: " << std::to_string(optical.get_rgb().blue - optical.get_rgb().red) << " AMBIENT PROXIMITY: " << ambientProximity << " AMBIENT DIFFERENCE: " << ambientColorDiff << "\n";
		//std::cout << "LED PWM" << optical.get_led_pwm() << "\n";
		//std::cout << stopDriverIntake << "\n";
    //std::cout << "distance: " << wallDist << "\n";
    //std::cout << "ladybrown velocity: " << ladybrown2.get_actual_velocity() << "\n";
        pros::delay(50);
        
        // Add a way to break the loop if needed
        if (pros::competition::is_disabled()) break;
    }
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  brakeModeHold();

	ColorLoopActive = true;
  colorFiltrationActive = true;
	intakeUnstuckActivated = true;

	if (!LBLoopActive) {
    pros::Task lb_task(LBLoop);
  }

	ladybrown1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  ladybrown2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  ladybrown2.tare_position();
  
  //pros::Task logger_task(logger);
  //skills();
  //ringWalk(allianceColorBlue);
  //RingRush6(allianceColorBlue);
  //stateSoloAwp(allianceColorBlue);
  //safeFourRing(allianceColorBlue);
  //newMogoRush(allianceColorBlue);
  //barcbotsMogoRush(allianceColorBlue);
  worldsMogoRush(allianceColorBlue);

  // ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
//pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  //brakeModeCoast();

  if (!LBLoopActive) { 
		pros::Task lb_task(LBLoop);
	}
  runStart = pros::millis();
	ladybrown1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  ladybrown2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  pros::Task logger_task(logger);
	pros::Task temp_task(checkTemp);
	
	ColorLoopActive = true; // starts inactive until tested ambient colors
  colorFiltrationActive = true;

	intakeUnstuckActivated = true;
	
  
  callLBReset();
  LBState = REST;
  //skillsMacro();
  chassis.opcontrol_curve_buttons_toggle(false);

  while (true) {

    //runArcadeDrive();
      	// Activate Intake Logic
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    //   double tempFahrenheit = intake.get_temperature() * 9.0 / 5.0 + 32.0;
    //   master.set_text(0, 0, "Intake Temp: " + std::to_string(tempFahrenheit) + "F");
    // } else {
    //   master.set_text(0, 0, "Avg Temp: " + std::to_string(averageTempFahrenheit) + "F");
    // }

		setIntakeMotors();
		// Activate Doinker Logic
		setDoinker();
      	// Activate Mogo Logic
		setMogoMotors();

    setIntakeLift();

		checkLBBroken();
    // Gives you some extras to make EZ-Template ezier
    //ez_template_extras();

    if (calledMoveBackForAWS) {
      chassis.pid_drive_set(-7, 1500, false, false); // move back
      chassis.pid_wait();
      ChangeLBState(FULLEXTENDED); // extend ladybrown
      calledMoveBackForAWS = false;
    }

    //chassis.opcontrol_tank();  // Tank control
     chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

void checkTemp() {
	int count;
	double totalTemp;
    while (true) {
        totalTemp = 0.0;
        count = 0;
        std::vector<double> left_side_temps = left_side_motors.get_temperature_all();
        std::vector<double> right_side_temps = right_side_motors.get_temperature_all();
        //std::cout << "Left Side Temps: " << left_side_temps[0] << ", " << left_side_temps[1] << ", " << left_side_temps[2] << "\n";
        //std::cout << "Right Side Temps: " << right_side_temps[0] << ", " << right_side_temps[1] << ", " << right_side_temps[2] << "\n";
        
        
          for (int i = 0; i < left_side_temps.size(); i++) {
            totalTemp += left_side_temps[i];
            count++;
          }
          for (int i = 0; i < right_side_temps.size(); i++) {
            totalTemp += right_side_temps[i];
            count++;
          }

          if (count < 6) {
            master.set_text(0, 0, "MOTOR UNPLUGGED");
            master.rumble("---");
          }
        if (IMU.get_heading() == NAN) {
          master.set_text(0, 0, "IMU unplugged.");
          pros::delay(250);
          master.rumble("---");
			  }

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			double tempFahrenheit = intake.get_temperature() * 9.0 / 5.0 + 32.0;
			master.set_text(0, 0, "Intake Temp: " + std::to_string(tempFahrenheit) + "F");
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			double tempFarenheit = ((ladybrown1.get_temperature() + ladybrown2.get_temperature()) / 2.0) * 9.0 / 5.0 + 32.0;
			master.set_text(0, 0, "LB Temp: " + std::to_string(tempFarenheit) + "F");
		} else {
			double averageTempCelsius = totalTemp / 6;
			averageTempFahrenheit = averageTempCelsius * 9.0f / 5.0 + 32.0;
      //std::cout << "Average Temp: " << averageTempFahrenheit << "\n";
      //master.clear_line(0);
			master.set_text(0, 0, "Avg Temp: " + std::to_string(averageTempFahrenheit) + "F");
		}
		pros::delay(500);
    }
}