#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor RightChassis = motor(PORT9, ratio18_1, false);
motor LeftChassis = motor(PORT2, ratio18_1, true);
controller Controller1 = controller(primary);
motor ArmMotor1 = motor(PORT8, ratio36_1, true);
motor ArmMotor2 = motor(PORT3, ratio36_1, false);
motor RightChassis1 = motor(PORT10, ratio18_1, false);
motor LeftChassis1 = motor(PORT1, ratio18_1, true);

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