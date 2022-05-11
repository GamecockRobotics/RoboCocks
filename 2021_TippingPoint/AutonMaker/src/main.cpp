
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>

std::ofstream ofs;

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

bool logging = false;
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
  while (true) {
    if (Controller1.ButtonA.pressing() && Brain.SDcard.isInserted() &&
        !logging) {

      Brain.Screen.print("logging true\n");

      logging = true;
    } else if (Controller1.ButtonA.pressing()) {

      Brain.Screen.print("SD Card is not inserted\n");
    } else if (Controller1.ButtonB.pressing() && logging) {
      Brain.Screen.print("logging: false\n");
    }

    LeftChassis.spin(forward, Controller1.Axis2.value(), percent);
    LeftChassis1.spin(forward, Controller1.Axis2.value(), percent);
    RightChassis.spin(forward, Controller1.Axis3.value(), percent);
    RightChassis1.spin(forward, Controller1.Axis3.value(), percent);
    if (logging && Brain.SDcard.isInserted()) {
      ofs << "LeftChassis.spin("
          << (LeftChassis.direction() == forward ? "forward" : "reverse")
          << ", " << LeftChassis.value() << ", percent);\n";
      ofs << "LeftChassis1.spin("
          << (LeftChassis1.direction() == forward ? "forward" : "reverse")
          << ", " << LeftChassis1.value() << ", percent);\n";
      ofs << "RightChassis.spin("
          << (RightChassis.direction() == forward ? "forward" : "reverse")
          << ", " << RightChassis.value() << ", percent);\n";
      ofs << "RightChassis1.spin("
          << (RightChassis1.direction() == forward ? "forward" : "reverse")
          << ", " << RightChassis1.value() << ", percent);\n";
    }

    if (Controller1.ButtonR1.pressing()) {
      ArmMotor1.spin(forward);
      ArmMotor2.spin(forward);
      if (logging) {
        ofs << "ArmMotor1.spin(forward);";
        ofs << "ArmMotor2.spin(forward);";
      }
    } else if (Controller1.ButtonR2.pressing()) {
      ArmMotor1.spin(reverse);
      ArmMotor2.spin(reverse);
      if (logging) {
        ofs << "ArmMotor1.spin(reverse);";
        ofs << "ArmMotor2.spin(reverse);";
      }
    } else {
      ArmMotor1.stop(hold);
      ArmMotor2.stop(hold);
      if (logging) {
        ofs << "ArmMotor1.stop(hold);";
        ofs << "ArmMotor2.stop(hold);";
      }
    }

    wait(20, msec);
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
