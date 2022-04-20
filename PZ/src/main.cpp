

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
enum intakeDirection { intake, outtake, stopped };
intakeDirection intakeState = stopped;

const double WHEEL_DIAMETER = 3.25;
const double CHASSIS_GEAR_RATIO = 60.0 / 36.0;

bool clawState;
// bool clawState2;
// const float GEAR_DIAMETER = 3.5;
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
  clawState = true;
  frontClaw.set(clawState);

  RightArm.setStopping(hold);
  LeftArm.setStopping(hold);

  LeftClamp.setStopping(hold);
  RightClamp.setStopping(hold);

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
void frontGrab(bool frontClawState) {
  clawState = frontClawState;
  frontClaw.set(frontClawState);
}

void drive(double dist) {
  double errorL = dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 360;
  double errorR = dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 360;
  double prevErrorL = errorL;
  double prevErrorR = errorR;
  double totalErrorL = 0;
  double totalErrorR = 0;
  const double threshold = 10.0;
  const float kp = 0.05;
  const float kd = 0.012;
  const float ki = 0.25;
  bool left = true;
  bool right = true;
  LeftChassis0.setRotation(0, degrees);
  RightChassis0.setRotation(0, degrees);
  while (left || right) {
    int speedL = kp * errorL + kd * (prevErrorL - errorL) + ki * totalErrorL;
    int speedR = kp * errorR + kd * (prevErrorR - errorR) + ki * totalErrorR;
    if (left) {
      LeftChassis0.spin(forward, speedL, percent);
      LeftChassis1.spin(forward, speedL, percent);
      LeftChassis2.spin(forward, speedL, percent);
    } else {
      LeftChassis0.stop(hold);
      LeftChassis1.stop(hold);
      LeftChassis2.stop(hold);
    }
    if (right) {
      RightChassis0.spin(forward, speedR, percent);
      RightChassis1.spin(forward, speedR, percent);
      RightChassis2.spin(forward, speedR, percent);
    } else {
      RightChassis0.stop(hold);
      RightChassis1.stop(hold);
      RightChassis2.stop(hold);
    }
    wait(200, msec);
    prevErrorL = errorL;
    prevErrorR = errorR;
    errorL = dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 360 -
             LeftChassis0.rotation(degrees);
    errorR = dist * M_1_PI / WHEEL_DIAMETER * CHASSIS_GEAR_RATIO * 360 -
             RightChassis0.rotation(degrees);
    totalErrorL = totalErrorL + ((fabs(errorL) < 10) ? errorL : 0);
    totalErrorR = totalErrorR + ((fabs(errorR) < 10) ? errorR : 0);
    left = fabs(errorL) > threshold || fabs(prevErrorL) > threshold;
    right = fabs(errorR) > threshold || fabs(prevErrorR) > threshold;
  }

  LeftChassis0.stop(hold);
  LeftChassis1.stop(hold);
  LeftChassis2.stop(hold);
  RightChassis0.stop(hold);
  RightChassis1.stop(hold);
  RightChassis2.stop(hold);
  Brain.Screen.print("done");
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  drive(96);
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

void leftDrive(double speed) {
  if (fabs(speed) > 5) {
    LeftChassis0.spin(forward, speed, percent);
    LeftChassis1.spin(forward, speed, percent);
    LeftChassis2.spin(forward, speed, percent);
  } else {
    LeftChassis0.stop(coast);
    LeftChassis1.stop(coast);
    LeftChassis2.stop(coast);
  }
}

void rightDrive(double speed) {
  if (fabs(speed) > 5) {
    RightChassis0.spin(forward, speed, percent);
    RightChassis1.spin(forward, speed, percent);
    RightChassis2.spin(forward, speed, percent);
  } else {
    RightChassis0.stop(coast);
    RightChassis1.stop(coast);
    RightChassis2.stop(coast);
  }
}

void claw() {
  if (clawState) {
    clawState = false;
    frontClaw.set(clawState);
    wait(100, msec);
  } else {
    clawState = true;
    frontClaw.set(clawState);
    wait(100, msec);
  }
}

void usercontrol(void) {
  // User control code here, inside the loop

  Controller1.ButtonA.pressed(toggleIntake);
  Controller1.ButtonB.pressed(toggleOuttake);

  Controller1.ButtonUp.pressed(claw);

  while (true) {

    leftDrive(Controller1.Axis3.value());
    rightDrive(Controller1.Axis2.value());

    if (Controller1.ButtonL1.pressing()) {
      LeftClamp.spin(forward, 100, percent);
      RightClamp.spin(forward, 100, percent);
    } else if (Controller1.ButtonL2.pressing()) {
      LeftClamp.spin(reverse, 100, percent);
      RightClamp.spin(reverse, 100, percent);
    } else {
      LeftClamp.stop(hold);
      RightClamp.stop(hold);
    }

    if (Controller1.ButtonR1.pressing()) {
      LeftArm.spin(forward, 100, percent);
      RightArm.spin(forward, 100, percent);
    } else if (Controller1.ButtonR2.pressing()) {
      LeftArm.spin(reverse, 100, percent);
      RightArm.spin(reverse, 100, percent);
    } else {
      LeftArm.stop(hold);
      RightArm.stop(hold);
    }
    // Brain.Screen.print("Butts");

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
