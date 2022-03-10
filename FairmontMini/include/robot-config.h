using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LeftArm;
extern motor RightArm;
extern motor LeftChassis0;
extern motor LeftChassis1;
extern motor LeftChassis2;
extern motor RightChassis0;
extern motor RightChassis1;
extern motor RightChassis2;
extern motor FrontClaw;
extern motor BackClaw;
extern motor Intake;
extern motor BackClaw2;
extern inertial Gyro;
extern controller Controller2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );