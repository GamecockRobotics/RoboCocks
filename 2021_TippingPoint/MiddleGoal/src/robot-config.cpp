#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftChassis = motor(PORT1, ratio18_1, true);
motor rightChassis = motor(PORT10, ratio18_1, false);
controller Controller1 = controller(primary);
motor ArmMotor1 = motor(PORT2, ratio36_1, false);
motor ArmMotor2 = motor(PORT9, ratio36_1, true);

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