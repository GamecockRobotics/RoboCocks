/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "math.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftArm              motor         1               
// RightArm             motor         10              
// LeftChassis0         motor         5               
// LeftChassis1         motor         4               
// LeftChassis2         motor         3               
// RightChassis0        motor         19              
// RightChassis1        motor         20              
// RightChassis2        motor         17              
// FrontClaw            motor         2               
// BackClaw             motor         7               
// Intake               motor         9               
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
enum intakeDirection { intake, outtake, stopped };
intakeDirection intakeState = stopped;

const int WHEEL_RADIUS = 2;

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

  BackClaw.resetPosition();
  FrontClaw.resetPosition();
  LeftArm.resetPosition();
  RightArm.resetPosition();
  LeftChassis0.resetPosition();
  LeftChassis1.resetPosition();
  LeftChassis2.resetPosition();
  RightChassis0.resetPosition();
  RightChassis1.resetPosition();
  RightChassis2.resetPosition();
  BackClaw.setStopping(hold);
  FrontClaw.setStopping(hold);
  LeftArm.setStopping(brake);
  RightArm.setStopping(brake);
  // RightChassis0.setStopping(hold);
  // RightChassis1.setStopping(hold);
  // RightChassis2.setStopping(hold);
  // LeftChassis0.setStopping(hold);
  // LeftChassis1.setStopping(hold);
  // LeftChassis2.setStopping(hold);
  LeftChassis0.setVelocity(100, percent);
  LeftChassis1.setVelocity(100, percent);
  LeftChassis2.setVelocity(100, percent);
  RightChassis0.setVelocity(100, percent);
  RightChassis1.setVelocity(100, percent);
  RightChassis2.setVelocity(100, percent);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void driveForward(int dist) {
  
  LeftChassis0.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, false);
  LeftChassis1.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, false);
  LeftChassis2.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, false);
  RightChassis0.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, false);
  RightChassis1.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, false);
  RightChassis2.spinFor(dist/WHEEL_RADIUS*M_1_PI, turns, true);
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

void autonomous() {
  Brain.Screen.print("Running Auton");
  driveForward(24);
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

void leftChassisSpin(int speed, brakeType type = coast) {
  if (abs(speed) > 5) {
    LeftChassis0.spin(forward, speed, percent);
    LeftChassis1.spin(forward, speed, percent);
    LeftChassis2.spin(forward, speed, percent);
  } else {
    LeftChassis0.stop(type);
    LeftChassis1.stop(type);
    LeftChassis2.stop(type);
  }
}
void rightChassisSpin(int speed, brakeType type = coast) {
  if (abs(speed) > 5) {
    RightChassis0.spin(forward, speed, percent);
    RightChassis1.spin(forward, speed, percent);
    RightChassis2.spin(forward, speed, percent);
  } else {
    RightChassis0.stop(type);
    RightChassis1.stop(type);
    RightChassis2.stop(type);
  }
}
void moveArm(bool up, bool down, int speed = 100, brakeType type = hold) {
  if (Controller1.ButtonLeft.pressing()) {
    LeftArm.spin(forward, speed, percent);
    RightArm.spin(forward, speed, percent);
  } else if (Controller1.ButtonDown.pressing()) {
    LeftArm.spin(reverse, speed, percent);
    RightArm.spin(reverse, speed, percent);
  } else {
    LeftArm.stop(type);
    RightArm.stop(type);
  }
}
void toggleIntake() {
  intakeState = (intakeState == intake ? stopped : intake);
  if (intakeState == intake) {
    Intake.spin(forward, 100, percent);
  } else {
    Intake.stop(coast);
  }
}
void toggleOuttake() {
  intakeState = (intakeState == outtake ? stopped : outtake);
  if (intakeState == outtake) {
    Intake.spin(reverse, 100, percent);
  } else {
    Intake.stop(coast);
  }
}
void frontClaw(bool close, bool open) {
  if (close) {
    FrontClaw.spin(forward, 80, percent);
  } else if (open) {
    FrontClaw.spin(reverse, 80, percent);
  } else {
    FrontClaw.stop(hold);
  }
}
void backClaw(bool close, bool open) {
  if (close) {
    BackClaw.spin(forward, 80, percent);
  } else if (open) {
    BackClaw.spin(reverse, 80, percent);
  } else {
    BackClaw.stop(hold);
  }
}

void usercontrol(void) {
  // User control code here, inside the loop
  bool aPressed = false;
  bool bPressed = false;
  while (true) {
    // Control for chassis
    leftChassisSpin(Controller1.Axis3.value());
    rightChassisSpin(Controller1.Axis2.value());
    // Control for Lift
    moveArm(Controller1.ButtonLeft.pressing(),
            Controller1.ButtonDown.pressing());
    // Control for Front Claw
    frontClaw(Controller1.ButtonL1.pressing(), 
            Controller1.ButtonL2.pressing());
    // Control for Back Claw
    backClaw(Controller1.ButtonR1.pressing(), 
            Controller1.ButtonR2.pressing());

    // Control for the Intake
    if (Controller1.ButtonA.pressing()) {
      if (!aPressed) {
        toggleIntake();
      }
      aPressed = true;
      bPressed = false;
    } else if (Controller1.ButtonB.pressing()) {
      if (!bPressed) {
        toggleOuttake();
      }
      aPressed = false;
      bPressed = true;
    } else {
      aPressed = false;
      bPressed = false;
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