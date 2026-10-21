// Two High Stakes routines written against LemLib 0.5.
#include "main.h"
#include "lemlib/api.hpp"

extern lemlib::Chassis chassis;
extern pros::Motor intake;
extern pros::adi::DigitalOut clamp;

ASSET(ladder_txt);

// Solo AWP from the positive side. Red is written out; blue mirrors x.
void soloAwp(bool isBlue) {
    const int s = isBlue ? -1 : 1;
    chassis.setPose(-55 * s, -12, isBlue ? 90 : 270);

    // back into the goal and clamp it
    chassis.moveToPoint(-24 * s, -24, 2000, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();
    clamp.set_value(true);
    intake.move(127);

    // two rings below the goal
    chassis.turnToPoint(-24 * s, -48, 1000);
    chassis.moveToPoint(-24 * s, -46, 1500);
    chassis.swingToHeading(isBlue ? 90 : 270, isBlue ? lemlib::DriveSide::RIGHT : lemlib::DriveSide::LEFT, 800,
                           {.minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPose(-48 * s, -10, 0, 2500, {.lead = 0.4});

    // touch the ladder
    chassis.turnToHeading(isBlue ? 225 : 135, 800,
                          {.direction = isBlue ? AngularDirection::CW_CLOCKWISE : AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-18 * s, -18, 1500, {.maxSpeed = 60});
    chassis.waitUntilDone();
    intake.move(0);
}

// Start of a skills run: back along a path.jerryio path into the first goal,
// then push it into the corner.
void skills() {
    chassis.setPose(-60, 0, 270);
    intake.move(127);
    chassis.follow(ladder_txt, 12, 4000, false);
    chassis.waitUntil(30);
    clamp.set_value(true);
    chassis.waitUntilDone();

    uint32_t start = pros::millis();
    while (pros::millis() - start < 600) {
        chassis.tank(-80, -80);
        pros::delay(10);
    }
    chassis.tank(0, 0);
    clamp.set_value(false);
    chassis.moveToPoint(-48, -40, 1500);

    // waits on a sensor: the importer can't know when this ends
    while (intake.get_torque() < 0.5) pros::delay(10);
    chassis.turnToHeading(180, 1000);
}
