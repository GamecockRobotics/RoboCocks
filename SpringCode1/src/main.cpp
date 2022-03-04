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
// BackLeftChassis1     motor         20              
// BackLeftChassis2     motor         19              
// FrontLeftChassis     motor         18              
// BackRightChassis1    motor         10              
// BackRightChassis2    motor         9               
// FrontRightChassis    motor         8               
// Controller1          controller                    
// RightArm             motor         1               
// LeftArm              motor         2               
// BackClaw             motor         3               
// FrontClaw            motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

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

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    /*
    BackLeftChassis1.spin(forward, Controller1.Axis3.value(), percent);
    BackLeftChassis2.spin(forward, Controller1.Axis3.value(), percent);
    FrontLeftChassis.spin(forward, Controller1.Axis3.value(), percent);
    BackRightChassis1.spin(forward, -Controller1.Axis2.value(), percent);
    BackRightChassis2.spin(forward, -Controller1.Axis2.value(), percent);
    FrontRightChassis.spin(forward, -Controller1.Axis2.value(), percent);
    */
    
    BackLeftChassis1.spin(forward, Controller1.Axis1.value()+Controller1.Axis3.value(), percent);
    BackLeftChassis2.spin(forward, Controller1.Axis1.value()+Controller1.Axis3.value(), percent);
    FrontLeftChassis.spin(forward, Controller1.Axis1.value()+Controller1.Axis3.value(), percent);
    BackRightChassis1.spin(forward, Controller1.Axis1.value()-Controller1.Axis3.value(), percent);
    BackRightChassis2.spin(forward, Controller1.Axis1.value()-Controller1.Axis3.value(), percent);
    FrontRightChassis.spin(forward, Controller1.Axis1.value()-Controller1.Axis3.value(), percent);

    
    if(Controller1.ButtonR1.pressing()){
      RightArm.spin(forward);
      LeftArm.spin(forward);
    } else if (Controller1.ButtonR2.pressing()){
      RightArm.spin(reverse);
      LeftArm.spin(reverse);
    } else {
      RightArm.stop(hold);
      LeftArm.stop(hold);
    }

    if(Controller1.ButtonL1.pressing()){
      FrontClaw.spin(forward);
    } else if (Controller1.ButtonL2.pressing()){
      FrontClaw.spin(reverse);
    } else {
      FrontClaw.stop(hold);
    }

    if(Controller1.ButtonA.pressing()){
      BackClaw.spin(forward);
    } else if (Controller1.ButtonB.pressing()){
      BackClaw.spin(reverse);
    } else {
      BackClaw.stop(hold);
    }




    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
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
