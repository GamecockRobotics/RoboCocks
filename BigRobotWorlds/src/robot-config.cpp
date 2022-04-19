#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor driveFrontLeft = motor(PORT17, ratio18_1, true);
motor driveMiddleLeft = motor(PORT18, ratio18_1, true);
motor driveBackLeft = motor(PORT19, ratio18_1, true);
motor driveFrontRight = motor(PORT8, ratio18_1, false);
motor driveMiddleRight = motor(PORT7, ratio18_1, false);
motor driveBackRight = motor(PORT5, ratio18_1, false);
inertial Gyro = inertial(PORT9);
motor backClawLeft = motor(PORT16, ratio18_1, true);
motor backClawRight = motor(PORT6, ratio18_1, false);
motor frontLiftRight = motor(PORT4, ratio18_1, false);
motor frontLiftLeft = motor(PORT15, ratio18_1, true);
motor Intake = motor(PORT1, ratio18_1, false);
digital_out leftPiston = digital_out(Brain.ThreeWirePort.A);
digital_out rightPiston = digital_out(Brain.ThreeWirePort.B);

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