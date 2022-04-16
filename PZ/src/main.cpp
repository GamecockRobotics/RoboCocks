// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Gyro                 inertial      15              
// LeftChassis0         motor         1               
// LeftClamp            motor         2               
// RightClamp           motor         9               
// LeftChassis1         motor         12              
// LeftArm              motor         13              
// LeftChassis2         motor         14              
// RightArm             motor         17              
// RightChassis1        motor         18              
// RightChassis2        motor         19              
// RightChassis0        motor         10              
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// driveFrontLeft       motor         20              
// driveMiddleLeft      motor         3               
// driveBackLeft        motor         2               
// driveFrontRight      motor         11              
// driveMiddleRight     motor         8               
// driveBackRight       motor         9               
// Gyro                 inertial      12              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
enum intakeDirection { intake, outtake, stopped };
intakeDirection intakeState = stopped;

const int WHEEL_DIAMETER = 4;
bool clawState;
bool clawState2;
//const float GEAR_DIAMETER = 3.5;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void leftDrive(double speed) {
  if (fabs(speed) > 5) {
    LeftChassis0.spin(forward, speed, percent);
    LeftChassis1.spin(forward, speed, percent);
    LeftChassis2.spin(forward, speed, percent);
  } else {
    LeftChassis0.stop();
    LeftChassis1.stop();
    LeftChassis2.stop();
  }
}

void rightDrive(double speed) {
   if (fabs(speed) > 5) {
    RightChassis0.spin(forward, speed, percent);
    RightChassis1.spin(forward, speed, percent);
    RightChassis2.spin(forward, speed, percent);
  } else {
    RightChassis0.stop();
    RightChassis1.stop();
    RightChassis2.stop();
  }
}

void usercontrol(void) {
  // User control code here, inside the loop
  
  while (true) {
    leftDrive(Controller1.Axis3.value());
    rightDrive(Controller1.Axis2.value());

    if (Controller1.ButtonL1.pressing()) {
    LeftClamp.spin(forward);
    RightClamp.spin(forward);
  } else if (Controller1.ButtonL2.pressing()) {
    LeftClamp.spin(reverse);
    RightClamp.spin(reverse);
  }
  else {
    LeftClamp.setBrake(hold);
    RightClamp.setBrake(hold);
  }


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
