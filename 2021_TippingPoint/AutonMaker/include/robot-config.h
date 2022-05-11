using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RightChassis;
extern motor LeftChassis;
extern controller Controller1;
extern motor ArmMotor1;
extern motor ArmMotor2;
extern motor RightChassis1;
extern motor LeftChassis1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );