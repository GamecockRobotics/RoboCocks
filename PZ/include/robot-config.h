using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern inertial Gyro;
extern motor LeftChassis0;
extern motor LeftClamp;
extern motor RightChassis0;
extern motor LeftChassis1;
extern motor LeftArm;
extern motor LeftChassis2;
extern motor RightArm;
extern motor RightChassis1;
extern motor RightChassis2;
extern motor RightClamp;
extern motor Intake;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );