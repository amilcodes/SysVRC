// robot.json: the JSON reader, the drivetrain fields, and the plant using them.
#include <string>

#include "check.hpp"
#include "visbot/visbot.hpp"
#include "visbot/robot_spec.hpp"

using namespace visbot;

TEST(json_reads_the_basics) {
    std::string err;
    const Json j = Json::parse(R"({"a": 1.5, "b": [true, null, "x\"y"], "c": {"d": -2e1}, "e": "A"})", &err);
    EXPECT_TRUE(err.empty());
    EXPECT_NEAR(j["a"].number(), 1.5, 1e-12);
    EXPECT_TRUE(j["b"].items().size() == 3);
    EXPECT_TRUE(j["b"].items()[0].boolean());
    EXPECT_TRUE(j["b"].items()[1].isNull());
    EXPECT_TRUE(j["b"].items()[2].string() == "x\"y");
    EXPECT_NEAR(j["c"]["d"].number(), -20.0, 1e-12);
    EXPECT_TRUE(j["e"].string() == "A");
    EXPECT_TRUE(j["missing"].isNull());
    EXPECT_TRUE(j["a"]["not an object"].isNull());
}

TEST(json_says_where_it_broke) {
    std::string err;
    Json::parse(R"({"a": 1,})", &err);
    EXPECT_TRUE(err.find("byte") != std::string::npos);
    Json::parse(R"([1, 2)", &err);
    EXPECT_TRUE(!err.empty());
    Json::parse(R"({"a": 1} x)", &err);
    EXPECT_TRUE(err.find("trailing") != std::string::npos);
}

TEST(spec_reads_the_drivetrain) {
    const auto r = parseRobotSpec(R"({
        "name": "test bot",
        "footprint": {"width": 14, "length": 16},
        "mass_lb": 18,
        "drive": {"wheel_diameter": 2.75, "cartridge_rpm": 600, "ratio": 0.8, "track_width": 11.5,
                  "motors_per_side": 3, "motor": "11W"},
        "mechanisms": [{"id": "intake", "kind": "intake"}]
    })");
    EXPECT_TRUE(r.ok());
    EXPECT_TRUE(r.spec.name == "test bot");
    EXPECT_NEAR(r.spec.physical.wheelDiameterIn, 2.75, 1e-12);
    EXPECT_NEAR(r.spec.physical.wheelRpm, 480.0, 1e-9);           // 600 * 0.8
    EXPECT_NEAR(r.spec.physical.trackWidthIn, 11.5, 1e-12);
    EXPECT_NEAR(r.spec.physical.robotLengthIn, 16.0, 1e-12);
    EXPECT_NEAR(r.spec.encoderScale, 1.0, 1e-12);                  // no code block: code matches
    EXPECT_TRUE(r.spec.json.find("\"mechanisms\"") != std::string::npos);
}

TEST(code_that_disagrees_with_the_robot_scales_the_encoders) {
    // CAD says 480 rpm wheels, the ez::Drive constructor says 450: EZ
    // undercounts by 450/480.
    const auto r = parseRobotSpec(R"({"drive": {"wheel_rpm": 480, "cartridge_rpm": 600},
                                      "code": {"wheel_rpm": 450, "wheel_diameter": 3.25}})");
    EXPECT_TRUE(r.ok());
    EXPECT_NEAR(r.spec.encoderScale, 450.0 / 480.0, 1e-12);
    EXPECT_NEAR(r.spec.declared.wheelRpm, 450.0, 1e-12);
}

TEST(heavier_or_faster_drives_respond_slower) {
    const auto base = parseRobotSpec(R"({"mass_lb": 15, "drive": {"wheel_diameter": 3.25, "wheel_rpm": 450, "motors_per_side": 3}})");
    const auto heavy = parseRobotSpec(R"({"mass_lb": 22, "drive": {"wheel_diameter": 3.25, "wheel_rpm": 450, "motors_per_side": 3}})");
    const auto four = parseRobotSpec(R"({"mass_lb": 15, "drive": {"wheel_diameter": 3.25, "wheel_rpm": 450, "motors_per_side": 2}})");
    const auto given = parseRobotSpec(R"({"drive": {"tau_s": 0.2}})");
    EXPECT_NEAR(base.spec.motorTauSec, 0.12, 1e-9);
    EXPECT_TRUE(heavy.spec.motorTauSec > base.spec.motorTauSec);
    EXPECT_TRUE(four.spec.motorTauSec > base.spec.motorTauSec);
    EXPECT_NEAR(given.spec.motorTauSec, 0.2, 1e-12);
}

TEST(bad_specs_are_rejected_with_a_reason) {
    EXPECT_TRUE(!parseRobotSpec("{").ok());
    EXPECT_TRUE(!parseRobotSpec(R"({"name": "no drive"})").ok());
    const auto neg = parseRobotSpec(R"({"drive": {"wheel_diameter": -3}})");
    EXPECT_TRUE(!neg.ok() && neg.errors[0].find("wheel_diameter") != std::string::npos);
    const auto wide = parseRobotSpec(R"({"footprint": {"width": 12}, "drive": {"track_width": 15}})");
    EXPECT_TRUE(!wide.ok());
    EXPECT_TRUE(!loadRobotSpec("/nonexistent/robot.json").ok());
}

TEST(spec_json_is_safe_inside_a_script_tag) {
    const auto r = parseRobotSpec(R"({"name": "</script><b>", "drive": {}})");
    EXPECT_TRUE(r.ok());
    EXPECT_TRUE(r.spec.json.find("</") == std::string::npos);
    std::string err;
    const Json back = Json::parse(r.spec.json, &err);   // and it round-trips
    EXPECT_TRUE(err.empty() && back["name"].string() == "</script><b>");
}

TEST(a_faster_robot_covers_more_ground) {
    // Same open-loop command, two drivetrains: the one geared for more rpm
    // goes further over a couple of seconds, though it takes longer to get up
    // to speed.
    const auto slow = parseRobotSpec(R"({"drive": {"wheel_diameter": 3.25, "wheel_rpm": 360}})");
    const auto fast = parseRobotSpec(R"({"drive": {"wheel_diameter": 3.25, "wheel_rpm": 600}})");
    auto travel = [](const RobotSpec& s) {
        PlantParams pp;
        pp.motorTauSec = s.motorTauSec;
        DiffDrivePlant plant(s.physical, pp);
        plant.reset({0, 0, 0});
        for (int i = 0; i < 240; ++i) plant.step({127, 127}, 1.0 / 120.0);
        return plant.truth().y;
    };
    EXPECT_TRUE(travel(fast.spec) > 1.4 * travel(slow.spec));
    EXPECT_TRUE(fast.spec.motorTauSec > slow.spec.motorTauSec);
}

TEST_MAIN()
