#include "selection.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

// Initialize static member
AutonomousSelector* AutonomousSelector::instance = nullptr;

// Implementation of the singleton constructor
AutonomousSelector::AutonomousSelector() : currentRoutine(3) { // Default to Disabled
    // Initialize selector UI
    pros::lcd::initialize();
    updateDisplay();
}

// Implement the rest of the member functions
// ...existing code...

AutonomousSelector* AutonomousSelector::getInstance() {
    if (instance == nullptr) {
        instance = new AutonomousSelector();
    }
    return instance;
}

// Define the selector variable
AutonomousSelector* selector = nullptr;

void AutonomousSelector::nextRoutine() {
    int next = currentRoutine + 1;
    if (next >= routineCount) next = 0;
    currentRoutine = next;
    updateDisplay();
}

void AutonomousSelector::previousRoutine() {
    int prev = currentRoutine - 1;
    if (prev < 0) prev = routineCount - 1;
    currentRoutine = prev;
    updateDisplay();
}

void AutonomousSelector::toggleAllianceColor() {
    allianceColorBlue = !allianceColorBlue;
    updateDisplay();
}

void AutonomousSelector::updateDisplay() {
    std::cout <<"Currrent Routine: " << routineNames[currentRoutine] << "\n";
    std::cout << "routine count: " << routineCount << "\n";
    // Clear each line individually since there's no clear() function
    pros::lcd::clear_line(1);
    pros::lcd::clear_line(2);
    pros::lcd::clear_line(3);
    pros::lcd::clear_line(4);
    
    pros::lcd::set_text(1, "Selected Autonomous:");
    pros::lcd::set_text(2, routineNames[currentRoutine]);
    pros::lcd::set_text(3, routineNotes[currentRoutine]);
    
    // Display competition status
    std::string status = "Status: ";
    if (pros::competition::is_connected()) {
        status += "Connected";
        if (pros::competition::is_autonomous()) {
            status += " (Autonomous)";
        } else if (pros::competition::is_disabled()) {
            status += " (Disabled)";
        } else {
            status += " (Opcontrol)";
        }
    } else {
        status += "Not Connected";
    }
    pros::lcd::set_text(4, status);

    // Display alliance color
    pros::lcd::set_text(5, allianceColorBlue ? "Alliance: Blue" : "Alliance: Red");
}

void AutonomousSelector::runSelectedAutonomous() {
    // Set brake modes before running autonomous

    switch (currentRoutine) {
        case 0:
            //disruptRingRush(allianceColorBlue);
            safeRingSide(allianceColorBlue);
            break;
        case 1:
            simpleRing(allianceColorBlue);
            break;
        case 2:
            newMogoRush(allianceColorBlue);
            break;
        case 3:
            //simpleMogo(allianceColorBlue);
            safeFourRing(allianceColorBlue);
            break;
        case 4:
            verySimpleMogo(allianceColorBlue);
            break;
        case 5:
            allianceColorBlue = false;
            skills();
            break;
        case 6:
            break;
        default:
            break;
    }
}

void initializeSelector() {
    selector = AutonomousSelector::getInstance();
    
    // Register button callbacks for the LCD using correct PROS API
    pros::lcd::register_btn0_cb([]() {
        if(selector) selector->previousRoutine(); 
    });
    
    pros::lcd::register_btn1_cb([]() {
        if(selector) selector->toggleAllianceColor();
    });

    pros::lcd::register_btn2_cb([]() {
        if(selector) selector->nextRoutine();
    });
}