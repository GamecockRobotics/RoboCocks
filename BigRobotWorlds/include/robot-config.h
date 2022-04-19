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
extern motor backClawLeft;
extern motor backClawRight;
extern motor frontLiftRight;
extern motor frontLiftLeft;
extern motor Intake;
extern digital_out leftPiston;
extern digital_out rightPiston;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );