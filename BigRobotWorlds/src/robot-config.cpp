#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor driveFrontLeft = motor(PORT20, ratio18_1, false);
motor driveMiddleLeft = motor(PORT3, ratio18_1, false);
motor driveBackLeft = motor(PORT2, ratio18_1, false);
motor driveFrontRight = motor(PORT11, ratio18_1, false);
motor driveMiddleRight = motor(PORT8, ratio18_1, false);
motor driveBackRight = motor(PORT9, ratio18_1, false);
inertial Gyro = inertial(PORT12);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}