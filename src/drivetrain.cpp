#include "vex.h"

using namespace vex;

void MecanumDrive(float driveSpeed, float turnSpeed, float strafeSpeed)
{
  double forward = driveSpeed;
  double sideways = 0.0  - strafeSpeed;
  double turn = turnSpeed;

  int rightFrontMotorSpeed = forward - turn + sideways;
  int leftFrontMotorSpeed = forward + turn - sideways;
  int rightRearMotorSpeed = forward + turn + sideways;
  int leftRearMotorSpeed = forward - turn - sideways;

  RightFrontDriveSmart.setVelocity(rightFrontMotorSpeed, percent);
  LeftFrontDriveSmart.setVelocity(leftFrontMotorSpeed, percent);
  LeftRearDriveSmart.setVelocity(rightRearMotorSpeed, percent);
  RightRearDriveSmart.setVelocity(leftRearMotorSpeed, percent);

  RightFrontDriveSmart.spin(vex::forward);
  LeftFrontDriveSmart.spin(vex::forward);
  LeftRearDriveSmart.spin(vex::forward);
  RightRearDriveSmart.spin(vex::forward);

}
