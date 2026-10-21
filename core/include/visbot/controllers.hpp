// visbot/controllers.hpp: the controller for a routine's drive library.
#pragma once

#include <memory>

#include "visbot/constants.hpp"
#include "visbot/lemlib.hpp"
#include "visbot/missions.hpp"
#include "visbot/motion.hpp"

namespace visbot {

/// `declared` is what the code tells EZ about the drivetrain. LemLib routines
/// carry their own settings (MissionDef::lem), so it's unused for them.
inline std::unique_ptr<AutonController> makeController(const MissionDef& def, const RobotParams& declared,
                                                       const DriveGains& gains = {}) {
    if (def.lib == Library::LemLib) return std::make_unique<LemController>(def.lem);
    return std::make_unique<MotionController>(declared, gains);
}

/// The drivetrain a LemLib project describes to lemlib::Drivetrain, on top of
/// `base` for everything LemLib doesn't know (size, mass).
inline RobotParams lemDrivetrain(const LemConfig& c, RobotParams base = {}) {
    base.trackWidthIn = c.trackWidth;
    base.wheelDiameterIn = c.wheelDiameter;
    base.wheelRpm = c.rpm;
    return base;
}

}  // namespace visbot
