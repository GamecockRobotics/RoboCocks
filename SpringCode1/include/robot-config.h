using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor BackLeftChassis1;
extern motor BackLeftChassis2;
extern motor FrontLeftChassis;
extern motor BackRightChassis1;
extern motor BackRightChassis2;
extern motor FrontRightChassis;
extern controller Controller1;
extern motor RightArm;
extern motor LeftArm;
extern motor BackClaw;
extern motor FrontClaw;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );