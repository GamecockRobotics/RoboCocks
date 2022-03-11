#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftArm = motor(PORT1, ratio18_1, true);
motor RightArm = motor(PORT10, ratio18_1, false);
motor LeftChassis0 = motor(PORT5, ratio18_1, false);
motor LeftChassis1 = motor(PORT4, ratio18_1, true);
motor LeftChassis2 = motor(PORT3, ratio18_1, false);
motor RightChassis0 = motor(PORT19, ratio18_1, true);
motor RightChassis1 = motor(PORT20, ratio18_1, false);
motor RightChassis2 = motor(PORT17, ratio18_1, true);
motor FrontClaw = motor(PORT2, ratio18_1, false);
motor BackClaw = motor(PORT7, ratio18_1, false);
motor Intake = motor(PORT9, ratio18_1, false);
motor BackClaw2 = motor(PORT11, ratio18_1, false);
inertial Gyro = inertial(PORT16);
controller Controller2 = controller(partner);

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