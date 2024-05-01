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

using namespace vex;

// A global instance of competition
competition Competition;

void MecanumDrive(float driveSpeed, float turnSpeed, float strafeSpeed = 0.0);

// define your global instances of motors and other devices here
event checkGreen = event();

#define ISRIGHT(x)   ((x) > 220)
#define ISLEFT(x)  ((x) < 100)
#define ISFAR(x)    ((x) < 180)
#define ISNEAR(x)   ((x) > 220)
#define DETECTSIZE 13

bool bGreenFound = false;
bool bGreenCentered = false;
int greenCount;
int greenX, greenY;
int greenH, greenW;
int greenDiagSize;
float greenAngle;
int greenDistMM;
const float GamePieceHeightMM = 6.0 * 2.54 * 10.0;
const float CameraHFOV = 67.0;
const float CameraVFOV = CameraHFOV * 3.0 / 4.0;
const float CameraHRES = 316.0;
const float CameraVRES = 212.0;

struct tVisionSample {
  int x, y;
  int size;
  int count;
};

struct tVisionStats {
  float xMean, xVar;
  float yMean, yVar;
  float sizeMean, sizeVar;
  int maxSize;
  float countMean, countVar;
  int totalValid;
};

#define SAMPLE_WINDOW 100
tVisionSample visionSamples[SAMPLE_WINDOW];
int curVisionSample = 0;
tVisionStats visionStats;

void calcVisionStats()
{
  int validSamples = 0;
  int32_t xTotal = 0;
  int32_t yTotal = 0;
  int32_t sizeTotal = 0;
  int32_t countTotal = 0;
  int maxSize = 0;

  for (int i = 0; i < SAMPLE_WINDOW; i++) {
    countTotal += visionSamples[i].count;
    if (visionSamples[i].count > 0) {
      validSamples++;
      xTotal += visionSamples[i].x;
      yTotal += visionSamples[i].y;
      sizeTotal += visionSamples[i].size;
      if (visionSamples[i].size > maxSize) maxSize = visionSamples[i].size;
    } else {
    }
  }

  float xMean = (float) xTotal / (float) validSamples;
  float yMean = (float) yTotal / (float) validSamples;
  double sizeMean = (double) sizeTotal / (double) validSamples;
  float countMean = (float) countTotal / (float) SAMPLE_WINDOW;

  float xVar = 0.0;
  float yVar = 0.0;
  double sizeVar = 0.0;
  float countVar = 0.0;

  for (int i = 0; i < SAMPLE_WINDOW; i++) {
    countVar += ((float) visionSamples[i].count - countMean) * ((float) visionSamples[i].count - countMean);
    if (visionSamples[i].count > 0) {
      xVar += ((float) visionSamples[i].x - xMean) * ((float) visionSamples[i].x - xMean);
      yVar += ((float) visionSamples[i].y - yMean) * ((float) visionSamples[i].y - yMean);
      sizeVar += ((double) visionSamples[i].size - sizeMean) * ((double) visionSamples[i].size - sizeMean);
    } else {
    }
  }

  xVar = fabsf(xVar / (float) validSamples);
  yVar = fabsf(yVar / (float) validSamples);
  sizeVar = fabs(sizeVar / (double) validSamples);
  countVar = fabsf(countVar / (float) SAMPLE_WINDOW);

  visionStats.xMean = xMean;
  visionStats.xVar = xVar;
  visionStats.yMean = yMean;
  visionStats.yVar = yVar;
  visionStats.sizeMean = (float) sizeMean;
  visionStats.sizeVar = (float) sizeVar;
  visionStats.maxSize = maxSize;
  visionStats.countMean = countMean;
  visionStats.countVar = countVar;
  visionStats.totalValid = validSamples;
}

uint32_t lastPrintTime = 0;
bool bIsSampling = true;
int visionRunTime = 0;
uint32_t loopTime = 0;
int loopCount = 0;
int dsDistance = -1;

void hasGreenCallback() {

  uint32_t startTime = vex::timer::system();

  int objectCount = Vision8.takeSnapshot(Vision8__GREENBOX1);
  bool dsDetected = DistanceSensor.isObjectDetected();
  if (dsDetected) dsDistance = (int) DistanceSensor.objectDistance(mm);
  else dsDistance = -1;

  greenCount = objectCount;
  // int objectCount = Vision8.takeSnapshot(4);
  if (objectCount > 0) {
    vision::object triball = Vision8.largestObject;
    greenX = triball.centerX;
    greenY = triball.centerY;
    greenH = triball.height;
    greenW = triball.width;

    float fGreenHeight = triball.height;
    // assume pixels are linear angle for now
    float fGreenAngle = (greenX - CameraHRES / 2.0) *  CameraHFOV / CameraHRES;
    greenAngle = fGreenAngle;
    float fMinDistMM = GamePieceHeightMM / sin(CameraVFOV * (2.0 * 3.14159 / 360.0));
    float fEstDistMM = GamePieceHeightMM / sin(CameraVFOV * (fGreenHeight / CameraVRES) * (2.0 * 3.14159 / 360.0)) ;
    greenDistMM = (int) fEstDistMM;
    bGreenCentered = (!ISLEFT(greenX) && !ISRIGHT(greenX)) ? true : false;
    greenDiagSize = int(sqrtf(triball.width * triball.height) + 0.5);
    bGreenFound = (greenDiagSize > DETECTSIZE) ? true : false;
    if (bIsSampling) {
      visionSamples[curVisionSample].x = triball.centerX;
      visionSamples[curVisionSample].y = triball.centerY;           
      visionSamples[curVisionSample].size = greenDiagSize;           
      visionSamples[curVisionSample].count = objectCount;
    }           
  } else {
    bGreenFound = false;
    bGreenCentered = false;
    if (bIsSampling) {
      visionSamples[curVisionSample].count = objectCount;    
    }
  }
  if (bIsSampling) {
  curVisionSample++;
    if (curVisionSample >= SAMPLE_WINDOW) {
      calcVisionStats();
      curVisionSample = 0;
    }
  }

  uint32_t endTime = vex::timer::system();
  if (endTime - startTime > visionRunTime) visionRunTime = endTime - startTime;

}

void printVisionStats() {

  uint32_t thisTime = vex::timer::system();
  bool printThis = false;
  if (thisTime - lastPrintTime > 1000) {
    printThis = true;
    lastPrintTime = thisTime;
  }

  if (!printThis) return;

  Brain.Screen.setFont(mono40);
  Brain.Screen.clearLine(1, black);
  Brain.Screen.setCursor(Brain.Screen.row(), 1);
  Brain.Screen.setCursor(1, 1);
 
  if (greenCount > 0) {
    Brain.Screen.print("%d Green Objects Found", greenCount);
    Brain.Screen.newLine();
    Brain.Screen.print("%0.1f | %3d,%3d | %3dx%3d\n",
      greenAngle, greenX, greenY,
      greenW, greenH);
    printf("angle: %0.1f, dist: %.1f/%f, center: %3d,%3d, dim: %3dx%3d\n",
      greenAngle, greenDistMM / (2.54 * 10.0), (dsDistance > 0) ? (float) dsDistance / (2.54 * 10.0) : -1.0, greenX, greenY,
      greenW, greenH); 
  } else {
    if (printThis) {
      Brain.Screen.print("No Green Object");
    }
  }
  if (bIsSampling) {
      printf("vision stats: %d x(%.2f, %0.2f) y(%.2f, %0.2f) s(%.2f, %0.2f, %d) c(%.2f, %0.2f)\n",
        visionStats.totalValid,
        visionStats.xMean, visionStats.xVar,
        visionStats.yMean, visionStats.yVar,
        visionStats.sizeMean, visionStats.sizeVar, visionStats.maxSize,
        visionStats.countMean, visionStats.countVar);
  }
}

void runGyroTest()
{
 //double startHeading = Drivetrain.heading(vex::degrees);
  //Brain.Screen.print("%.0lf", startHeading);
  //Brain.Screen.newLine();

  Drivetrain.setDriveVelocity(100.0, vex::percent);
  Drivetrain.setTurnVelocity(100.0, vex::percent);

  for (int i = 0; i < 10; i++) {
    printf("heading: %0.2lf\n", InertialSensor.heading(degrees));
    Drivetrain.setTimeout(10, seconds);
    Drivetrain.turnToHeading(90.0, vex::rotationUnits::deg, 50.0, vex::velocityUnits::pct, true);
    Drivetrain.stop();
    // Drivetrain.turnFor(right, 90.0, vex::rotationUnits::deg, 50.0, vex::velocityUnits::pct, true);
    printf("heading: %0.2lf\n", InertialSensor.heading(degrees));

    wait(3, seconds);

    Drivetrain.setTimeout(10, seconds);
    Drivetrain.turnToHeading(0.0, vex::rotationUnits::deg, 50.0, vex::velocityUnits::pct, true);
    Drivetrain.stop();
    printf("heading: %0.2lf\n", InertialSensor.heading(degrees));

    wait(3, seconds);
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
}

typedef enum EDRIVESTATE {
  ALLSTOP = 0,
  SEARCHING_START = 1,
  SEARCHING = 2,
  TRACKING = 3,
  DRIVE_START = 4,
  DRIVE = 5
};

bool bCancelVisionTest = false;

void runVisionTest()
{
  bool bIsTurning = false;
  bool bIsDriving = false;
  bool bWasLeft = false;

  EDRIVESTATE curState = ALLSTOP;
  EDRIVESTATE nextState;

  int loopCount = 0;
  int startCount = 0;

  while(!bCancelVisionTest) {
    // TODO: Add last known location to direction of turn
    // TODO: Optimized color learning?
    if (curState == ALLSTOP) {
      if (bGreenFound) nextState = TRACKING;
      else nextState = SEARCHING_START;

    } else if (curState == SEARCHING_START) {
      Drivetrain.setTurnVelocity(50.0, vex::percent);    
      Drivetrain.turnFor((bWasLeft) ? -360.0 : 360.0, vex::degrees, false);
      startCount = loopCount;
      nextState = SEARCHING;

    } else if (curState == SEARCHING) {
      if (bGreenFound) {
        Drivetrain.stop(coast);
        nextState = TRACKING;
        // BUGBUG: Never completes
        //      } else if (Drivetrain.isDone()) { 
      } else if (loopCount - startCount > 400) {
        Drivetrain.stop(coast);
        nextState = ALLSTOP;
      }

    } else if (curState == TRACKING) {
      float driveSpeed = 0.0;
      float turnSpeed = 0.0;
      if (bGreenFound) {
        if (bGreenCentered) {
          if (greenAngle < -1.0) {
            turnSpeed = -5.0;
            bWasLeft = true; 
          } else if (greenAngle > 1.0) {
            turnSpeed = 5.0;
            bWasLeft = false; 
          }
          int calcDistMM = 0;
          if (dsDistance > 0) {
            if (dsDistance < 250) calcDistMM = dsDistance;
            else calcDistMM = (dsDistance + greenDistMM) / 2; 
          } else {
            calcDistMM = greenDistMM;
          }
          if (calcDistMM > 500) {
            driveSpeed = 30.0;
          }else if (calcDistMM > 250) {
            driveSpeed = 20.0;
          } else if (calcDistMM < 200) {
            driveSpeed = -5.0;
          }
        } else {
          if (ISLEFT(greenX)) {
            turnSpeed = -15.0;    
            bWasLeft = true;
          } else if (ISRIGHT(greenX)) {
            turnSpeed = 15.0; 
            bWasLeft = false;
          }
        }
        nextState = TRACKING;
      } else {
        // Drivetrain.stop(brake);
        driveSpeed = 0.0;
        turnSpeed = 0.0;
        nextState = ALLSTOP;
      }

      MecanumDrive(driveSpeed, turnSpeed);

    } else if (curState == TRACKING) {
      printf("T\n");
 
    } else if (curState == DRIVE_START) {
      printf("DS\n");
      nextState = TRACKING;

    } else if (curState == DRIVE) {

    }

    wait(50, msec);
    curState = nextState;
    loopCount++;
  }

  Drivetrain.stop(coast);
  
}

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
      printf("%0.2lf, %0.2lf, %0.2lf\n", InertialSensor.roll(), InertialSensor.pitch(), InertialSensor.yaw());
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
  UIGYROTEST = 3,
};

EUIMODE uiMode = UIIDLE;

void OnButtonAPressed()
{
  printf("Running Gyro Test\n");
  uiMode = UIGYROTEST;
  runGyroTest();
}

int VisionThread()
{
  // TODO: Camera finish power-up on initial boot
  printf("Vision Thread ...\n");
  checkGreen(hasGreenCallback);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    //uint32_t startTime = vex::timer::system();
    // Takes about 4-6ms
    checkGreen.broadcastAndWait();
    //uint32_t endTime = vex::timer::system();
    //printf("time: %lu\n", endTime - startTime);
    this_thread::sleep_for(20);
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

  uiMode = UIIDLE;

  vex::thread vt = vex::thread(VisionThread);

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
      else if (UIDRIVERCONTROL) {
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("driver control");
        Controller1.Screen.newLine();
        Controller1.Screen.print("X: to cancel");
      }
      else if (UIVISIONTEST) {
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("vision test");
        Controller1.Screen.newLine();
        Controller1.Screen.print("B: to cancel");
      }
      else if (UIGYROTEST) {
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("gyro test");
        // Controller1.Screen.newLine();
        // Controller1.Screen.print("B: to cancel");
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
