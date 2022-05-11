

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

void turn(turnType dir, double deg) {
  float error = deg;
  float prevError = deg;
  float totalError = 0;
  const float threshold = 2.0;
  const float kp = 0.50;
  const float kd = 0.12;
  const float ki = 0.01;
  Gyro.setRotation(0, degrees);
  while (fabs(error) > threshold || fabs(prevError) > threshold) {
    int speed = kp * error + kd * (prevError - error) + ki * totalError;
    LeftChassis0.spin(dir == left ? reverse : forward, speed, percent);
    LeftChassis1.spin(dir == left ? reverse : forward, speed, percent);
    LeftChassis2.spin(dir == left ? reverse : forward, speed, percent);
    RightChassis0.spin(dir == right ? reverse : forward, speed, percent);
    RightChassis1.spin(dir == right ? reverse : forward, speed, percent);
    RightChassis2.spin(dir == right ? reverse : forward, speed, percent);
    wait(200, msec);
    prevError = error;
    error = deg - fabs(Gyro.rotation());

    totalError = totalError + (fabs(error) < 10 ? error : 0);
  }
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
}

void claw() {
  clawState = !clawState;
  frontClaw.set(clawState);
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  drive(44.5);
  claw();
  drive(-30);
  turn(right, 90);
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

int leftTarget = 0; 
int rightTarget = 0;

int driveChassis() {
  double kp = 0.6, kd = 0.24, ki = 0.0;
  double rightP = 0, leftP = 0, rightD = 0, leftD = 0, rightI = 0, leftI = 0;
  double prevRightSpeed = 0, prevLeftSpeed = 0, rightSpeed = 0, leftSpeed = 0;
  while (true) {
    rightI += ki*rightP/kp;
    rightP = kp*(rightTarget-rightSpeed);
    rightD = kd*(prevRightSpeed-rightSpeed);
    prevRightSpeed = rightSpeed;
    leftI += ki*rightP/kp;
    leftP = kp*(leftTarget-leftSpeed);
    leftD = kd*(prevLeftSpeed-leftSpeed);
    prevLeftSpeed = leftSpeed;
    if (rightSpeed != rightTarget) {
      if ((rightP + rightI + rightD) > 0 && rightSpeed < -50) {
        Brain.Screen.print(rightP);
        Brain.Screen.print(" ");
        Brain.Screen.print(rightI);
        Brain.Screen.print(" ");
        Brain.Screen.print(rightD);
        Brain.Screen.newLine();
      }
      rightSpeed += rightP + rightI + rightD;
    }
    if (leftSpeed != leftTarget) {
      leftSpeed += leftP + leftI + leftD;
    }
    RightChassis0.spin(forward, rightSpeed, percent);
    RightChassis1.spin(forward, rightSpeed, percent);
    RightChassis2.spin(forward, rightSpeed, percent);
    LeftChassis0.spin(forward, leftSpeed, percent);
    LeftChassis1.spin(forward, leftSpeed, percent);
    LeftChassis2.spin(forward, leftSpeed, percent);
    wait(10, msec);
  }
  return 0;
}

void usercontrol(void) {
  // User control code here, inside the loop

  Controller1.ButtonA.pressed(toggleIntake);
  Controller1.ButtonB.pressed(toggleOuttake);
  Controller1.ButtonUp.pressed(claw);
  task driveTask(driveChassis);
  while (true) {

    leftTarget = abs(Controller1.Axis3.value()) > 5  ? Controller1.Axis3.value():0;
    rightTarget = abs(Controller1.Axis2.value()) > 5 ? Controller1.Axis2.value():0;

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
    leftTarget = abs(Controller1.Axis3.value()) > 5  ? Controller1.Axis3.value():0;
    rightTarget = abs(Controller1.Axis2.value()) > 5 ? Controller1.Axis2.value():0;
    wait(100, msec);
  }
}
