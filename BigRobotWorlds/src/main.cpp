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
// driveFrontLeft       motor         16              
// driveMiddleLeft      motor         18              
// driveBackLeft        motor         19              
// driveFrontRight      motor         2               
// driveMiddleRight     motor         7               
// driveBackRight       motor         5               
// Gyro                 inertial      9               
// backClawLeft         motor         17              
// backClawRight        motor         8               
// frontLiftRight       motor         4               
// frontLiftLeft        motor         15              
// Intake               motor         1               
// claw                 digital_out   H               
// rightPiston          digital_out   B               
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
// bool clawState2;
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
  clawState = true;
  // clawState2 = true;
  claw.set(clawState);
  driveFrontLeft.setVelocity(100,percent);
  driveMiddleLeft.setVelocity(100,percent);
  driveBackLeft.setVelocity(100,percent);
  driveFrontRight.setVelocity(100,percent);
  driveMiddleRight.setVelocity(100,percent);
  driveBackRight.setVelocity(100,percent);

  backClawLeft.setVelocity(100,percent);
  backClawRight.setVelocity(100,percent);

  driveFrontLeft.setStopping(coast);
  driveMiddleLeft.setStopping(coast);
  driveBackLeft.setStopping(coast);
  driveFrontRight.setStopping(coast);
  driveMiddleRight.setStopping(coast);
  driveBackRight.setStopping(coast);

}

void driveForward(int dist, bool waiting = true) {
  
  driveFrontLeft.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveMiddleLeft.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveBackLeft.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveFrontRight.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveMiddleRight.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveBackRight.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, waiting);

  
}

void TurnLeft(float dist){
  driveFrontLeft.spinFor(-dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveMiddleLeft.spinFor(-dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveBackLeft.spinFor(-dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveFrontRight.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveMiddleRight.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveBackRight.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
}

void TurnRight(float dist){
  driveFrontLeft.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveMiddleLeft.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveBackLeft.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveFrontRight.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveMiddleRight.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
  driveBackRight.spinFor(dist/WHEEL_DIAMETER*M_1_PI, turns, false);
}

void frontGrab(bool frontClawState){
  clawState = frontClawState;
  claw.set(frontClawState);
}

void lift(float ang, bool waiting = false){
    frontLiftRight.spinFor(ang/7, turns, false);
    frontLiftLeft.spinFor(ang/7, turns, waiting);
}

void drive (double dist) {
  double errorL = dist/WHEEL_DIAMETER*M_1_PI;
  double errorR = dist/WHEEL_DIAMETER*M_1_PI;
  double errorA = 0;
  double prevErrorL = errorL;
  double prevErrorR = errorR;
  double prevErrorA = 0;
  double totalErrorL = 0;
  double totalErrorR = 0;
  double totalErrorA = 0;
  const double threshold = 2.0;
  const float kp = 0.50;
  const float kd = 0.12;
  const float ki = 0.01;
  const float kpa = 0.05;
  const float kda = 0.01;
  const float kia = 0.001;
  bool left = true;
  bool right = true;
  bool straight = true;
  driveBackLeft.setRotation(0, degrees);
  driveBackRight.setRotation(0, degrees);
  while (left || right || !straight) {
    Brain.Screen.print("I\n");
    int speedL = kp*errorL+kd*(prevErrorL-errorL) + ki*totalErrorL;// - (kpa*errorA + kda*(prevErrorA-errorA) +kia*totalErrorA);
    int speedR = kp*errorR+kd*(prevErrorR-errorR) + ki*totalErrorR;// + (kpa*errorA + kda*(prevErrorA-errorA) +kia*totalErrorA);
    if (left) {
      driveBackLeft.spin(forward, speedL, percent);
      driveMiddleLeft.spin(forward, speedL, percent);
      driveFrontLeft.spin(forward, speedL, percent);
    } else {
      driveBackLeft.stop(hold);
      driveMiddleLeft.stop(hold);
      driveFrontLeft.stop(hold);
    }
    if (right) {
      driveBackRight.spin(forward, speedR, percent);
      driveMiddleRight.spin(forward, speedR, percent);
      driveFrontRight.spin(forward, speedR, percent);
    } else {
      driveBackRight.stop(hold);
      driveMiddleRight.stop(hold);
      driveFrontRight.stop(hold);
    }
    wait(200, msec);
    prevErrorL = errorL;
    prevErrorR = errorR;
    prevErrorA = errorA;
    errorL = dist/WHEEL_DIAMETER*M_1_PI - driveBackLeft.rotation(degrees);
    errorR = dist/WHEEL_DIAMETER*M_1_PI - driveBackRight.rotation(degrees);
    errorA = fabs(Gyro.heading()) > 3 ? Gyro.heading() : 0;
    if (fabs(errorL) < 10) {
      totalErrorL = totalErrorL + errorL;
    }
    if (fabs(errorR) < 10) {
      totalErrorR = totalErrorR + errorR;
    }
    totalErrorA = totalErrorA + errorA;
    left = fabs(errorL) > threshold || fabs (prevErrorL) > threshold;
    right = fabs(errorR) > threshold || fabs (prevErrorR) > threshold;
  }
}

void chassisTurn (double deg, turnType dir) {
  float error = deg;
  float prevError = deg;
  float totalError = 0;
  const float threshold = 2.0;
  const float kp = 0.50; //0.5
  const float kd = 0.12; //0.12
  const float ki = 0.00;
  Gyro.setRotation(0, degrees);
  while (fabs(error) > threshold ||fabs (prevError) > threshold) {
    int speed = kp*error+kd*(prevError-error) + ki*totalError;
    driveFrontRight.spin(dir == left? forward:reverse, speed, percent);
    driveMiddleRight.spin(dir == left? forward:reverse, speed, percent);
    driveBackRight.spin(dir == left? forward:reverse, speed, percent);
    driveFrontLeft.spin(dir == right?forward:reverse, speed, percent);
    driveMiddleLeft.spin(dir == right?forward:reverse, speed, percent);
    driveBackLeft.spin(dir == right?forward:reverse, speed, percent);
    wait(200, msec);
    prevError = error;
    error = deg - fabs(Gyro.rotation());
    if (fabs(error) < 10) {
      totalError = totalError + error;
    }
  }

  driveFrontRight.stop();
  driveMiddleRight.stop();
  driveBackRight.stop();
  driveFrontLeft.stop();
  driveMiddleLeft.stop();
  driveBackLeft.stop();

}

void setMotorSpeed(float s){
  driveFrontLeft.setVelocity(s,percent);
  driveMiddleLeft.setVelocity(s,percent);
  driveBackLeft.setVelocity(s,percent);
  driveFrontRight.setVelocity(s,percent);
  driveMiddleRight.setVelocity(s,percent);
  driveBackRight.setVelocity(s,percent);
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
  //chassisTurn(90,right);
  drive(10000);
  lift(3,true);
  lift(-3);


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
void backClaw(){
  
}

void frontClaw(){
  if(clawState){
      clawState = false;
      claw.set(clawState);
      wait(100, msec);
    } else {
      clawState = true;
      claw.set(clawState);
      wait(100, msec);
    }
}

void leftChassisSpin(int speed) {
  if (abs(speed) > 5) {
    driveBackLeft.spin(forward, speed, percent);
    driveMiddleLeft.spin(forward, speed, percent);
    driveFrontLeft.spin(forward, speed, percent);
  } else {
    driveBackLeft.stop(coast);
    driveMiddleLeft.stop(coast);
    driveFrontLeft.stop(coast);
  }
}
void rightChassisSpin(int speed) {
  if (abs(speed) > 5) {
    driveBackRight.spin(forward, speed, percent);
    driveMiddleRight.spin(forward, speed, percent);
    driveFrontRight.spin(forward, speed, percent);
  } else {
    driveBackRight.stop(coast);
    driveMiddleRight.stop(coast);
    driveFrontRight.stop(coast);
  }
}

 const int speed = 100;

void usercontrol(void) {
  // User control code here, inside the loop
  bool aPressed = false;
  bool bPressed = false;
  Controller1.ButtonR1.pressed(frontClaw);
  while (1) {
    leftChassisSpin(Controller1.Axis3.value());
    rightChassisSpin(Controller1.Axis2.value());

    

    //Claw
    
    
    if(Controller1.ButtonLeft.pressing()){
      backClawLeft.spin(reverse, 100, percent);
      backClawRight.spin(reverse, 100, percent);
    } else if(Controller1.ButtonUp.pressing()){
      backClawLeft.spin(forward, 100, percent);
      backClawRight.spin(forward, 100, percent);
    } else{
      backClawLeft.stop(hold);
      backClawRight.stop(hold);
    }
    
    //Lift
    if(Controller1.ButtonL1.pressing()){
      frontLiftRight.spin(forward, speed, percent);
      frontLiftLeft.spin(forward, speed, percent);
    } else if(Controller1.ButtonL2.pressing()){
      frontLiftRight.spin(reverse, speed, percent);
      frontLiftLeft.spin(reverse, speed, percent);
    } else{
      frontLiftRight.stop(hold);
      frontLiftLeft.stop(hold);
    }

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
