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

bool bCancelDriverControl = false;

void DriverControl()
{
  double endHeading = InertialSensor.heading(vex::degrees);
  // double endHeading = Drivetrain.heading(vex::degrees);
  Brain.Screen.print("%.0lf", endHeading);
  Brain.Screen.newLine();
  printf("heading: %0.2lf, %0.2lf\n", InertialSensor.heading(degrees), Drivetrain.heading());
  printf("%0.2lf, %0.2lf, %0.2lf\n", InertialSensor.roll(), InertialSensor.pitch(), InertialSensor.yaw());

  int count = 0;

  while (!bCancelDriverControl) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    MecanumDrive(
      Controller1.Axis2.position(vex::percent),
      Controller1.Axis4.position(vex::percent),
      Controller1.Axis1.position(vex::percent)
      );

    // Drivetrain.arcade(forward, -turn);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
    count++;
    if (count == 50) {
      printf("heading: %0.2lf, %0.2lf\n", InertialSensor.heading(degrees), Drivetrain.heading());
      printf("roll: %0.2lf, pitch: %0.2lf, yaw: %0.2lf\n", InertialSensor.roll(), InertialSensor.pitch(), InertialSensor.yaw());
      printf("X: %0.2lf, Y: %0.2lf, Z: %0.2lf\n", InertialSensor.acceleration(vex::axisType::xaxis), InertialSensor.acceleration(vex::axisType::yaxis), InertialSensor.acceleration(vex::axisType::zaxis));
      count = 0;
    }
  }

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

  uiMode = UIIDLE;

  StartVision();

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
    printVisionStats();
  }
}
