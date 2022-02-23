#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor BackLeftChassis1 = motor(PORT20, ratio18_1, false);
motor BackLeftChassis2 = motor(PORT19, ratio18_1, true);
motor FrontLeftChassis = motor(PORT18, ratio18_1, false);
motor BackRightChassis1 = motor(PORT10, ratio18_1, false);
motor BackRightChassis2 = motor(PORT9, ratio18_1, true);
motor FrontRightChassis = motor(PORT8, ratio18_1, false);
controller Controller1 = controller(primary);

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