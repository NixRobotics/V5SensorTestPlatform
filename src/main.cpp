/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 10, D        
// ClawMotor            motor         3               
// ArmMotor             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"
#include "visionlib.h"
#include "drivetrain.h"

using namespace vex;

// A global instance of competition
competition Competition;

void OnButtonAPressed();

bool bCancelGyroTest = false;

void runGyroTest()
{
 //double startHeading = Drivetrain.heading(vex::degrees);
  //Brain.Screen.print("%.0lf", startHeading);
  //Brain.Screen.newLine();

  Drivetrain.setDriveVelocity(100.0, vex::percent);
  Drivetrain.setTurnVelocity(100.0, vex::percent);

  for (int i = 0; i < 2; i++) {
    printf("heading: %0.2lf\n", InertialSensor.heading(degrees));
    Drivetrain.setTimeout(10, seconds);
    Drivetrain.turnToHeading(90.0, vex::rotationUnits::deg, 50.0, vex::velocityUnits::pct, true);
    Drivetrain.stop();
    // Drivetrain.turnFor(right, 90.0, vex::rotationUnits::deg, 50.0, vex::velocityUnits::pct, true);
    printf("heading: %0.2lf\n", InertialSensor.heading(degrees));

    if (bCancelGyroTest) break;

    wait(3, seconds);

    if (bCancelGyroTest) break;

    Drivetrain.setTimeout(10, seconds);
    Drivetrain.turnToHeading(0.0, vex::rotationUnits::deg, 50.0, vex::velocityUnits::pct, true);
    Drivetrain.stop();
    printf("heading: %0.2lf\n", InertialSensor.heading(degrees));

    if (bCancelGyroTest) break;

    wait(3, seconds);

    if (bCancelGyroTest) break;

  }
  /*
  Drivetrain.turnFor(right, 90.0, vex::rotationUnits::deg, 50.0, vex::velocityUnits::pct, true);
  printf("heading: %0.2lf\n", InertialSensor.heading(degrees));
  wait(1, seconds);
  Drivetrain.turnFor(right, 90.0, vex::rotationUnits::deg, 50.0, vex::velocityUnits::pct, true);
  printf("heading: %0.2lf\n", InertialSensor.heading(degrees));
  wait(1, seconds);
  Drivetrain.turnFor(right, 90.0, vex::rotationUnits::deg, 50.0, vex::velocityUnits::pct, true);
  printf("heading: %0.2lf\n", InertialSensor.heading(degrees));
  wait(1, seconds);
  Drivetrain.turnFor(left, 360.0, vex::rotationUnits::deg, 50.0, vex::velocityUnits::pct, true);
  printf("heading: %0.2lf\n", InertialSensor.heading(degrees));
  /**/

  // Drivetrain.turn(left, 100.0, vex::velocityUnits::pct);
  // Drivetrain.turnFor(360.0, vex::degrees, false);
  // Drivetrain.turnFor(360.0, rotationUnits::deg, 100.0, velocityUnits::pct, false);
  // wait (2, seconds);
  // Drivetrain.stop(coast);
  // Drivetrain.turn(left, 25.0, vex::velocityUnits::pct);
  // Drivetrain.turnFor(360.0, rotationUnits::deg, 25.0, velocityUnits::pct, false);
  // Drivetrain.turnFor(360.0, vex::degrees, false);

  //double endHeading = Drivetrain.heading(vex::degrees);
  //Brain.Screen.print("%.0lf", endHeading);
  //Brain.Screen.newLine();  
  OnButtonAPressed();
}

typedef enum EAUTOMODE {
  AUTONONE = 0,
  AUTONORTH = 1,
  AUTOLEFT = 2
};

bool bCancelDriverControl = false;
bool bStraightenHeading = false;
bool bStraightenLeftSide = false;
bool bAutoCommandRunning = false;

double RAD_TO_DEG = 180.0 / M_PI;
double DEG_TO_RAD = M_PI / 180.0;

bool getWallAngle(float *out_wallAngle = NULL, float *out_wallDistance = NULL)
{
  float dsep = 15.0;
  float dfront = -1.0;
  float dback = -1.0;
  float ddelta = 0.0;
  bool turnLeft = false;
  bool turnRight = false;
  float wallAngle = 0.0;
  float wallDistance = 0.0;
  bool bValid = false;
  if (DistanceLeftFront.isObjectDetected()) dfront = DistanceLeftFront.objectDistance(distanceUnits::in);
  if (DistanceLeftRear.isObjectDetected()) dback = DistanceLeftRear.objectDistance(distanceUnits::in);
  // printf("Distance %f %f\n", dfront, dback);
  if ((dfront >= 0.0) && (dback >= 0.0) && (fabs(dfront - dback) < dsep)) {
    bValid = true;
    wallDistance = (dfront + dback) / 2.0;
    if (dfront > dback) {
      ddelta = dfront - dback;
      wallAngle = atanf(ddelta / dsep) * RAD_TO_DEG;
      turnLeft = true;
    } else if (dfront < dback) {
      ddelta = dback - dfront;
      wallAngle = atanf(ddelta / dsep) * RAD_TO_DEG;
      turnRight = true;
    }
  }

  if (out_wallDistance != NULL) *out_wallDistance = wallDistance;

  if (turnLeft) {
    // printf("TURN LEFT %f\n", wallAngle);
    if (out_wallAngle != NULL) *out_wallAngle = -wallAngle;
  } else if (turnRight) {
    // printf("TURN RIGHT %f\n", wallAngle);
    if (out_wallAngle != NULL) *out_wallAngle = wallAngle;
    return(wallAngle);
  }

  return bValid;
}


void DriverControl()
{
  double endHeading = InertialSensor.heading(vex::degrees);

  // double endHeading = Drivetrain.heading(vex::degrees);
  Brain.Screen.print("%.0lf", endHeading);
  Brain.Screen.newLine();
  printf("heading: %0.2lf, %0.2lf\n", InertialSensor.heading(degrees), Drivetrain.heading());
  printf("%0.2lf, %0.2lf, %0.2lf\n", InertialSensor.roll(), InertialSensor.pitch(), InertialSensor.yaw());
  
  // Controller1.Screen.setCursor(3, 1);
  // Controller1.Screen.print("H: %03d", (int) InertialSensor.heading(degrees));
  // Controller1.Screen.newLine();

  EAUTOMODE autoMode = AUTONONE;
  float holdHeading = 0.0;
  int count = 0;

  while (!bCancelDriverControl) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    bool bStartAutoCommand = false;

    if (autoMode == AUTONONE) {
      if (!bAutoCommandRunning) {
        if (bStraightenHeading) {
          autoMode = AUTONORTH;
          bStartAutoCommand = true;
        }
        else if (bStraightenLeftSide) {
          autoMode = AUTOLEFT;
          bStartAutoCommand = true;
        }
      }
    } else {
      if (!bAutoCommandRunning) {
        if (autoMode == AUTONORTH && !bStraightenHeading) autoMode = AUTONONE;
        else if (autoMode == AUTOLEFT && !bStraightenLeftSide) autoMode = AUTONONE;
      }
    }

    if (bStartAutoCommand) {
      if (autoMode == AUTONORTH) {
        bAutoCommandRunning = true;
        bStartAutoCommand = false;
        Drivetrain.stop(coast);
        Drivetrain.setTurnVelocity(50,percent);
        Drivetrain.turnToHeading(0.0,degrees,false);
      }
      else if (autoMode == AUTOLEFT) {
        bStartAutoCommand = false;
        float wallAngle, wallDistance;
        if (getWallAngle(&wallAngle, &wallDistance)) {
          bAutoCommandRunning = true;
          Drivetrain.stop(coast);
          Drivetrain.setTurnVelocity(50,percent);
          Drivetrain.turnFor(wallAngle,degrees,false);
        }
      }
    } else if (bAutoCommandRunning) {
      if (autoMode == AUTONORTH) {
        if (!Drivetrain.isTurning()) {
          Drivetrain.stop(coast);
          bAutoCommandRunning = false;
          holdHeading = Drivetrain.heading();
        }
      }
      else if (autoMode == AUTOLEFT) {
        if (!Drivetrain.isTurning()) {
          Drivetrain.stop(coast);
          bAutoCommandRunning = false;
          holdHeading = Drivetrain.heading();
        }
      }
    }

    float driveSpeed = Controller1.Axis3.position(vex::percent);
    float turnSpeed = Controller1.Axis4.position(vex::percent);
    float strafeSpeed = Controller1.Axis1.position(vex::percent);

    if (autoMode == AUTONORTH) {
      turnSpeed = 0.0;
      strafeSpeed = 0.0;
      float headingError = holdHeading - Drivetrain.heading();
      
      if (headingError > 180) headingError = headingError - 360.0;
      else if (headingError < -180) headingError = headingError + 360.0;


      turnSpeed = headingError;
      if (turnSpeed > 5.0) turnSpeed = 5.0;
      else if (turnSpeed < -5.0) turnSpeed = -5.0;
    }
    else if (autoMode == AUTOLEFT) {
      turnSpeed = 0.0;
      strafeSpeed = 0.0;
      float wallAngle, wallDistance;
      if (getWallAngle(&wallAngle, &wallDistance)) {
        float headingError = wallAngle;
        
        turnSpeed = headingError;
        if (turnSpeed > 10.0) turnSpeed = 10.0;
        else if (turnSpeed < -10.0) turnSpeed = -10.0;
      }
    }

    if (!bAutoCommandRunning) {
      MecanumDrive(driveSpeed, turnSpeed, strafeSpeed);
    }

    // Drivetrain.arcade(forward, -turn);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
    count++;
    if (count == 50) {

      double Ax = InertialSensor.acceleration(vex::axisType::xaxis);
      double Ay = InertialSensor.acceleration(vex::axisType::yaxis);
      double Az = InertialSensor.acceleration(vex::axisType::zaxis);
      double roll = 180.0 + atan2(Ay, Az) * RAD_TO_DEG;
      double pitch = -(180.0 + atan2(Ax, Az) * RAD_TO_DEG);
      double Ax_r = Ax * sin(pitch * DEG_TO_RAD);

      // printf("heading: %0.2lf, %0.2lf\n", InertialSensor.heading(degrees), Drivetrain.heading());
      // printf("roll: %0.2lf/%0.2lf, pitch: %0.2lf/%0.2lf, yaw: %0.2lf\n", InertialSensor.roll(), roll, InertialSensor.pitch(), pitch, InertialSensor.yaw());
      // printf("X: %0.5lf/%-.5lf, Y: %0.5lf, Z: %0.5lf\n", Ax, Ax_r, Ay, Az);

      int leftFrontDist = -1, leftRearDist = -1;
      if (DistanceLeftFront.isObjectDetected() && DistanceLeftFront.objectSize() == sizeType::large) {
        leftFrontDist = DistanceLeftFront.objectDistance(distanceUnits::mm);
      }
      if (DistanceLeftRear.isObjectDetected() && DistanceLeftRear.objectSize() == sizeType::large) {
        leftRearDist = DistanceLeftRear.objectDistance(distanceUnits::mm);
      }

      printf("F: %d, R: %d\n", leftFrontDist, leftRearDist);

      // Controller1.Screen.clearLine(3);
      // Controller1.Screen.setCursor(3, 1);
      // Controller1.Screen.print("H:%03d,F:%d,R:%d", (int) InertialSensor.heading(degrees), leftFrontDist, leftRearDist);
      // Controller1.Screen.newLine();

      count = 0;
    }
  }

}

void StartInertial()
{
  
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
bool bRobotInitDone = false;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  printf("pre_auton++\n");
  vexcodeInit();
  printf("pre_auton--\n");

  StartInertial();
  StartVision();

  bRobotInitDone = true;
  
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
  printf("autonomous\n");

  while (!bRobotInitDone) {
    wait(25, msec);
  }
  if (InertialSensor.isCalibrating()) return;

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

typedef enum EUIMODE {
  UINONE = -1,
  UIIDLE = 0,
  UIDRIVERCONTROL = 1,
  UIVISIONTEST = 2,
  UIVISIONOPT = 3,
  UIGYROTEST = 4,
};

EUIMODE uiMode = UIIDLE;

void OnButtonAPressed()
{
  static bool bIsGyroEnabled = false;
  static vex::thread gt;

  if (!bIsGyroEnabled) {
    printf("Running Gyro Test\n");
    bCancelGyroTest = false;
    gt = vex::thread(runGyroTest);
    bIsGyroEnabled = true;
    uiMode = UIGYROTEST;

  } else {
    bCancelGyroTest = true;
    gt.join();
    bIsGyroEnabled = false;
    uiMode = UIIDLE;
  }

}

void OnButtonBPressed()
{
  static bool bIsVisionEnabled = false;
  static vex::thread vt;

  if (!bIsVisionEnabled) {
    printf("Running Vision Test\n");
    bCancelVisionTest = false;
    vt = vex::thread(runVisionTest);
    bIsVisionEnabled = true;
    uiMode = UIVISIONTEST;
  } else {
    bCancelVisionTest = true;
    vt.join();
    bIsVisionEnabled = false;
    uiMode = UIIDLE;
  }
}

void OnButtonXPressed()
{
  static bool bIsDriverEnabled = false;
  static vex::thread dt;

  if (!bIsDriverEnabled) {
    printf("Driver Control\n");
    bCancelDriverControl = false;
    dt = vex::thread(DriverControl);
    bIsDriverEnabled = true;
    uiMode = UIDRIVERCONTROL;

  } else {
    bCancelDriverControl = true;
    dt.join();
    bIsDriverEnabled = false;
    uiMode = UIIDLE;
  }
}

void OnButtonUpPressed()
{
  if (bAutoCommandRunning) return;
  if (bStraightenHeading) bStraightenHeading = false;
  else {
    bStraightenHeading = true;
    bStraightenLeftSide = false;
  }
}

void OnButtonLeftPressed()
{
  if (bAutoCommandRunning) return;
  if (bStraightenLeftSide) bStraightenLeftSide = false;
  else {
    bStraightenLeftSide = true;
    bStraightenHeading = false;
  }
}


void OnButtonR1Pressed()
{
  NextObjectType();
}

#ifdef VISIONOPT
void OnButtonYPressed()
{
  if (!bIsVisionOpt) {
    printf("Vision Optimization On\n");
    bIsVisionOpt = true;
  } else {
    printf("Vision Optimization Off\n");
    bIsVisionOpt = false;
  }
}
#endif

EUIMODE lastUiMode = UINONE;

void usercontrol(void) {
  // User control code here, inside the loop
  printf("usercontrol\n");

  while (!bRobotInitDone) {
    wait(25, msec);
  }
  if (InertialSensor.isCalibrating()) return;

  Controller1.ButtonA.pressed(OnButtonAPressed);
  Controller1.ButtonB.pressed(OnButtonBPressed);
  Controller1.ButtonX.pressed(OnButtonXPressed);
  #ifdef VISIONOPT
  Controller1.ButtonY.pressed(OnButtonYPressed);
  #endif
  Controller1.ButtonUp.pressed(OnButtonUpPressed);
  Controller1.ButtonLeft.pressed(OnButtonLeftPressed);
  
  Controller1.ButtonR1.pressed(OnButtonR1Pressed);

  uiMode = UIIDLE;

//  StartInertial();
//  StartVision();

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    if (lastUiMode != uiMode) {
      if (uiMode == UIIDLE) {
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("A: gyro");
        Controller1.Screen.newLine();
        Controller1.Screen.print("B: vision");
        Controller1.Screen.newLine();
        Controller1.Screen.print("X: driver");
      }
      else if (uiMode == UIDRIVERCONTROL) {
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("driver control");
        Controller1.Screen.newLine();
        Controller1.Screen.print("X: to cancel");
        Controller1.Screen.newLine();
        Controller1.Screen.print("U: Nth, L: Wall");
      }
      else if (uiMode == UIVISIONTEST) {
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("vision test");
        Controller1.Screen.newLine();
        Controller1.Screen.print("B: to cancel");
      }
      else if (uiMode == UIGYROTEST) {
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("gyro test");
        Controller1.Screen.newLine();
        Controller1.Screen.print("A: to cancel");
      }
      lastUiMode = uiMode;
    }

    if (uiMode == UIVISIONTEST) {
      Controller1.Screen.setCursor(3, 1);
      Controller1.Screen.print("R1: %s  ", CurrentObject());
      Controller1.Screen.newLine();
    }
    
    wait(100, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


//
// Main will set up the competition functions and callbacks.
//
int main() {

  // Run the pre-autonomous function.
  pre_auton();
  // vexcodeInit();

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  printf("Initialization Complete\n");

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(1, seconds);
    getWallAngle();
    printVisionStats();
  }
}
