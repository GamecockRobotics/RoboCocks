using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor driveFrontLeft;
extern motor driveMiddleLeft;
extern motor driveBackLeft;
extern motor driveFrontRight;
extern motor driveMiddleRight;
extern motor driveBackRight;
extern inertial Gyro;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );