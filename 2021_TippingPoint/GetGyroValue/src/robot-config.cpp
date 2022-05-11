#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor driveFrontLeft = motor(PORT20, ratio18_1, true);
motor driveMiddleLeft = motor(PORT3, ratio18_1, true);
motor driveBackLeft = motor(PORT2, ratio18_1, true);
motor driveFrontRight = motor(PORT11, ratio18_1, false);
motor driveMiddleRight = motor(PORT8, ratio18_1, false);
motor driveBackRight = motor(PORT9, ratio18_1, false);
digital_out DigitalOutF = digital_out(Brain.ThreeWirePort.F);
inertial Gyro = inertial(PORT13);
motor RightLift = motor(PORT10, ratio18_1, false);
motor LeftLift = motor(PORT1, ratio18_1, true);
digital_out DigitalOutH = digital_out(Brain.ThreeWirePort.H);
motor Intake = motor(PORT6, ratio18_1, true);

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