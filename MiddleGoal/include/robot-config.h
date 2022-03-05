using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor leftChassis;
extern motor rightChassis;
extern controller Controller1;
extern motor ArmMotor1;
extern motor ArmMotor2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );