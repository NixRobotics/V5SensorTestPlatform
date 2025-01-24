#include "vex.h"
#include "visionlib.h"
#include "drivetrain.h"

using namespace vex;

using signature = vision::signature;
using code = vision::code;

// signature Vision8__BLUEBOX =
//     signature(1, -3441, -2785, -3113, 8975, 10355, 9665, 2.5, 0);
// signature Vision8__GREENBOX =
//     signature(2, -5767, -4965, -5366, -3803, -2861, -3332, 9.0, 0);
signature Vision8__GREENBOX1 =
    signature(1, -5113, -4239, -4676, -4311, -3567, -3938, 8.000, 0);
signature Vision8__SIG_1 = signature(1, 0, 0, 0, 0, 0, 0, 2.5, 0);
signature Vision8__SIG_2 = signature(2, 0, 0, 0, 0, 0, 0, 2.5, 0);
signature Vision8__SIG_3 = signature(3, 0, 0, 0, 0, 0, 0, 2.5, 0);
signature Vision8__SIG_4 = signature(4,  0, 0, 0, 0, 0, 0, 2.5, 0);
signature Vision8__SIG_5 = signature(5, 0, 0, 0, 0, 0, 0, 2.5, 0);
signature Vision8__SIG_6 = signature(6, 0, 0, 0, 0, 0, 0, 2.5, 0);
signature Vision8__SIG_7 = signature(7, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vision Vision8 = vision(PORT8);
vision Vision8 =
    vision(PORT8, 50, Vision8__SIG_1, Vision8__SIG_2, Vision8__SIG_3,
           Vision8__SIG_4, Vision8__SIG_5, Vision8__SIG_6, Vision8__SIG_7);

// define your global instances of motors and other devices here
event checkGreen = event();

#define ISRIGHT(x)   ((x) > 220)
#define ISLEFT(x)  ((x) < 100)
#define ISFAR(x)    ((x) < 180)
#define ISNEAR(x)   ((x) > 220)
#define DETECTSIZE 14

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
  float range;
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

//#define VISIONOPT 1
#ifdef VISIONOPT
bool bIsVisionOpt = false;
#endif
float fGreenRange = 0.0;
signature Vision8__TEMP;

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
  visionStats.range = fGreenRange;
}

uint32_t lastPrintTime = 0;
bool bIsSampling = true;
bool bSamplingQueued = false;
int visionRunTime = 0;
uint32_t loopTime = 0;
int loopCount = 0;
int dsDistance = -1;

void hasGreenCallback() {

  uint32_t startTime = vex::timer::system();

  int objectCount = Vision8.takeSnapshot(Vision8__TEMP);
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
    float fMinDistMM = GamePieceHeightMM / sin(CameraVFOV * (2.0 * M_PI / 360.0));
    float fEstDistMM = GamePieceHeightMM / sin(CameraVFOV * (fGreenHeight / CameraVRES) * (2.0 * M_PI / 360.0)) ;
    greenDistMM = (int) fEstDistMM;
    bGreenCentered = (!ISLEFT(greenX) && !ISRIGHT(greenX)) ? true : false;
    greenDiagSize = int(sqrtf(triball.width * triball.height) + 0.5);
    bGreenFound = (greenDiagSize > DETECTSIZE) ? true : false;
    if (bIsSampling && !bSamplingQueued) {
      visionSamples[curVisionSample].x = triball.centerX;
      visionSamples[curVisionSample].y = triball.centerY;           
      visionSamples[curVisionSample].size = greenDiagSize;           
      visionSamples[curVisionSample].count = objectCount;
    }           
  } else {
    bGreenFound = false;
    bGreenCentered = false;
    if (bIsSampling && !bSamplingQueued) {
      visionSamples[curVisionSample].count = objectCount;    
    }
  }
  if (bIsSampling && !bSamplingQueued) {
    curVisionSample++;
    if (curVisionSample >= SAMPLE_WINDOW) {
      calcVisionStats();
      curVisionSample = 0;
#ifdef VISIONOPT
      if (bIsVisionOpt) {
        fGreenRange += 0.25;
        if (fGreenRange > 20.0) fGreenRange = 0.0;
        signature Vision8__TEMP =
          signature(
            1,
            Vision8__GREENBOX1.uMin,
            Vision8__GREENBOX1.uMax,
            Vision8__GREENBOX1.uMean,
            Vision8__GREENBOX1.vMin,
            Vision8__GREENBOX1.vMax,
            Vision8__GREENBOX1.vMean,
            fGreenRange,
            0);
        Vision8.setSignature(Vision8__TEMP);
      }
#endif
      bSamplingQueued = true;
    }
  }

  uint32_t endTime = vex::timer::system();
  if (endTime - startTime > visionRunTime) visionRunTime = endTime - startTime;

}

// define VISION_VERBOSE  1

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
#ifdef VISION_VERBOSE
    printf("angle: %0.1f, dist: %.1f/%f, center: %3d,%3d, dim: %3dx%3d, runtime %dms\n",
      greenAngle, greenDistMM / (2.54 * 10.0), (dsDistance > 0) ? (float) dsDistance / (2.54 * 10.0) : -1.0, greenX, greenY,
      greenW, greenH,
      visionRunTime); 
#endif
  } else {
    if (printThis) {
      Brain.Screen.print("No Green Object");
    }
  }
  if (bIsSampling && bSamplingQueued) {
      printf("vision stats: %d %.2f x(%.2f, %0.2f) y(%.2f, %0.2f) s(%.2f, %0.2f, %d) c(%.2f, %0.2f)\n",
        visionStats.totalValid,
        visionStats.range,
        visionStats.xMean, visionStats.xVar,
        visionStats.yMean, visionStats.yVar,
        visionStats.sizeMean, visionStats.sizeVar, visionStats.maxSize,
        visionStats.countMean, visionStats.countVar);
      bSamplingQueued = false;
  }
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

int VisionThread()
{
  // TODO: Camera finish power-up on initial boot
  printf("Vision Thread ...\n");

  uint8_t red, green, blue;
  Vision8.getWhiteBalanceValues(&red, &green, &blue);
  printf("Camera white balance: %3d,%3d,%3d\n", red, green, blue);

  fGreenRange = Vision8__GREENBOX1.range;
  signature Vision8__TEMP =
    signature(
      1,
      Vision8__GREENBOX1.uMin,
      Vision8__GREENBOX1.uMax,
      Vision8__GREENBOX1.uMean,
      Vision8__GREENBOX1.vMin,
      Vision8__GREENBOX1.vMax,
      Vision8__GREENBOX1.vMean,
      fGreenRange,
      0);

  Vision8.setSignature(Vision8__TEMP);

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

void StartVision()
{
  vex::thread vt = vex::thread(VisionThread);
}

