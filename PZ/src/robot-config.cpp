#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
inertial Gyro = inertial(PORT15);
motor LeftChassis0 = motor(PORT1, ratio6_1, true);
motor LeftClamp = motor(PORT2, ratio18_1, false);
motor RightChassis0 = motor(PORT9, ratio6_1, false);
motor LeftChassis1 = motor(PORT12, ratio6_1, true);
motor LeftArm = motor(PORT13, ratio18_1, true);
motor LeftChassis2 = motor(PORT14, ratio6_1, true);
motor RightArm = motor(PORT17, ratio18_1, false);
motor RightChassis1 = motor(PORT18, ratio6_1, false);
motor RightChassis2 = motor(PORT19, ratio6_1, false);
motor RightClamp = motor(PORT10, ratio18_1, false);
motor Intake = motor(PORT5, ratio18_1, false);

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