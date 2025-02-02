#include "vex.h"
#include "visionlib.h"
#include "visionopt.h"
#include "drivetrain.h"

using namespace vex;

// using signature = vision::signature;
// using code = vision::code;

// signature Vision8__BLUEBOX =
//     signature(1, -3441, -2785, -3113, 8975, 10355, 9665, 2.5, 0);
// signature Vision8__GREENBOX =
//     signature(2, -5767, -4965, -5366, -3803, -2861, -3332, 9.0, 0);

/*
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
/**/

aivision::colordesc AIVision8__REDBOX(1, 238, 44, 125, 10, 0.2);
aivision::colordesc AIVision8__GREENBOX(2, 15, 195, 85, 10, 0.2);
aivision::colordesc AIVision8__BLUEBOX(3, 7, 56, 188, 10, 0.2);

// AI Vision Color Descriptions
aivision::colordesc AIVision8__GRNTRIBALL(1, 66, 168, 107, 25, 0.5);
aivision::colordesc AIVision8__REDDONUT(2, 199, 18, 101, 24, 0.34);
aivision::colordesc AIVision8__BLUEDONUT(3, 241, 86, 162, 14, 0.29);
// aivision::colordesc AIVision8__YELLOWMOGO(4, 168, 178, 0, 15, 0.2); // 168,178,0 / 96,109,0
aivision::colordesc AIVision8__YELLOWMOGO(4, 100, 150, 75, 12, 0.4); // 168,178,0 / 96,109,0

// Note: hue range is 1-40, saturation range is 0.1-1

#define COLMOGO
//#define COLREDDONUT
//#define COLBLUEDONUT
//#define COLGREENTRIBALL

enum gameElements {
  mobileGoal,
  redRing,
  blueRing,
};

// vex::aivision AIVision8(PORT8, AIVision8__GRNTRIBALL, AIVision8__REDDONUT, AIVision8__BLUEBOX, aivision::ALL_AIOBJS);
vex::aivision AIVision8(PORT8);

// define your global instances of motors and other devices here
event checkGreen = event();
event checkAIObj = event();

struct tObjectDescriptor {
  bool bFound = false;
  bool bCentered = false;
  int type = -1;
  int count;
  int x, y;
  int height, width;
  float angle;
  int distMM;
  int d1, d2, d3;
};

tObjectDescriptor colObj, aiObj;

// TODO: Actual detect size will be slightly smaller due to bounding box

const float TriballHeightMM = 6.0 * 2.54 * 10.0; // TODO
const bool bTriballDetectHeight = true;
#define TRIBALL_DETECTSIZE 14 // filter based on area

const float BlueRignWidthMM = 7.0 * 2.54 * 10.0; // TODO
const bool bBlueRingDetectHeight = false;
#define BLUERING_DETECTSIZE 14 // filter based on area

const float RedRignWidthMM = 7.0 * 2.54 * 10.0; // TODO
const bool bRedRingDetectHeight = false;
#define REDRING_DETECTSIZE 14 // filter based on area

const float MOGOHeightMM = 10.25 * 2.54 * 10.0; // TODO
#define MOGO_DETECTSIZE 14 // filter based on area

const float MOGOWidthMM = 7.0 * 2.54 * 10.0; // TODO
const bool bMogoDetectHeight = false;
#define MOGO_DETECTSIZE 8 // filter based on area

float colPysicalDimMM = 0.0;
bool bColDetecHeight = false;
int colDetectSize = 0;

const float CameraHFOV = 74.0;
const float CameraVFOV = 63.0;
const float CameraHRES = 320.0;
const float CameraVRES = 240.0;
const float CameraV_DegreePerPix = CameraVFOV / CameraVRES;

const float CameraHeightMM = 12.75 * 2.54 * 10.0;
const float CameraClosestMM = 6.85 * 2.54 * 10.0;
const float CameraNearAngle = atanf(CameraClosestMM/CameraHeightMM) * 360.0 / (2.0 * M_PI);

#define ISRIGHT(x)   ((x) > (160 + 60))
#define ISLEFT(x)  ((x) < (160 - 60))

int iColRed = 0, iColGreen = 0, iColBlue = 0, iColHue = 0;
float fColSat = 0.0;
aivision::colordesc* AIVision8__TEMP = NULL;

// TODO: Adjust for mounting height of camera
// TODO: Track closest object of certain type

uint32_t lastPrintTime = 0;
bool bIsSampling = true;
bool bSamplingQueued = false;
int visionRunTime = 0;
uint32_t loopTime = 0;
int loopCount = 0;
int dsDistance = -1;

void DetectObject(tObjectDescriptor *colObj, bool bDetectHeight, int detectSize)
{
    float fObjHeight = colObj->height;
    float fObjWidth = colObj->width;
    // assume pixels are linear angle for now
    float fAngle = (colObj->x - CameraHRES / 2.0) *  CameraHFOV / CameraHRES;
    colObj->angle = fAngle;
    // float fMinDistMM = TriballHeightMM / sin(CameraVFOV * (2.0 * M_PI / 360.0));
    float fCloseAngle = ((float) CameraVRES - (float) colObj->y - (float) colObj->height / 2.0) * CameraV_DegreePerPix;
    float fObjectAngle = CameraNearAngle + fCloseAngle;
    float fEstDistMM = CameraHeightMM * tanf(fObjectAngle * 2.0 * M_PI / 360.0);

    float fEstDistMM_2;
    if (bDetectHeight) {
      fEstDistMM_2 = colPysicalDimMM / sin(CameraVFOV * (fObjHeight / CameraVRES) * (2.0 * M_PI / 360.0)) ; // from camera
    } else {
      fEstDistMM_2 = colPysicalDimMM / sin(CameraHFOV * (fObjWidth / CameraHRES) * (2.0 * M_PI / 360.0)) ; // from camera
    }
    fEstDistMM_2 = sqrtf(fEstDistMM_2 * fEstDistMM_2 - CameraHeightMM * CameraHeightMM);

    // printf("%d, %d, %d, %d, %0.1f, %0.1f, %d\n", triball.centerX, triball.centerY, triball.width, triball.height, fEstDistMM, fEstDistMM_2, dsDistance);

    colObj->distMM = (int) fEstDistMM_2;
    colObj->bCentered = (!ISLEFT(colObj->x) && !ISRIGHT(colObj->x)) ? true : false;
    int diagSize = int(sqrtf(colObj->width * colObj->height) + 0.5);
    colObj->bFound = (diagSize > detectSize) ? true : false;

    colObj->d1 = fEstDistMM;
    colObj->d2 = fEstDistMM_2;
    colObj->d3 = dsDistance;
    
}

void hasGreenCallback() {

  uint32_t startTime = vex::timer::system();

  // AIVision8.takeSnapshot(AIVision8__TEMP);
  AIVision8.takeSnapshot(*AIVision8__TEMP);
  int  visObjectCount = AIVision8.objectCount;
  bool dsDetected = DistanceSensor.isObjectDetected();
  if (dsDetected) dsDistance = (int) DistanceSensor.objectDistance(mm);
  else dsDistance = -1;

  colObj.count = visObjectCount;
  // int objectCount = Vision8.takeSnapshot(4);
  if (colObj.count > 0) {
    aivision::object selectedObj = AIVision8.largestObject;

    colObj.x = selectedObj.centerX;
    colObj.y = selectedObj.centerY;
    colObj.height = selectedObj.height;
    colObj.width = selectedObj.width;

    DetectObject(&colObj, bColDetecHeight, colDetectSize);

    int diagSize = int(sqrtf(colObj.width * colObj.height) + 0.5);

    if (bIsSampling && !bSamplingQueued) {
      vsAddSample(colObj.count, colObj.x, colObj.y, diagSize, colObj.d1, colObj.d2, colObj.d3);
    }       
  } else {
    colObj.bFound = false;
    colObj.bCentered = false;
    if (bIsSampling && !bSamplingQueued) {
      vsAddSample(colObj.count);   
    }
  }
  if (bIsSampling && !bSamplingQueued) {
    if (vsUpdateSample()) {
      calcVisionStats(iColRed, iColGreen, iColBlue, iColHue, fColSat);
      if (voRun()) {
        iColRed = voRedCur;
        iColGreen = voGreenCur;
        iColBlue = voBlueCur;
        iColHue = voHueCur;
        fColSat = voSatCur;

        delete AIVision8__TEMP;
        AIVision8__TEMP = new aivision::colordesc(
            1,
            iColRed,
            iColGreen,
            iColBlue,
            (float) iColHue,
            fColSat
            );
        AIVision8.set(*AIVision8__TEMP);        
      }
      bSamplingQueued = true;
    }
  }

  uint32_t endTime = vex::timer::system();
  if (endTime - startTime > visionRunTime) visionRunTime = endTime - startTime;

}

int aiLastObjectType = -1;

struct tAIVisionSample {
  int type;
  float score;
};
tAIVisionSample aiSamples[SAMPLE_WINDOW];
int curAISample = 0;
bool bAISamplingQueued = false;

struct tAIStats {
  int redTotal, blueTotal, mogoTotal;
  float redScore, blueScore, mogoScore;
};
tAIStats aiStats;

void calcAIStats()
{
  int validSamples = 0;
  int32_t redTotal = 0;
  int32_t blueTotal = 0;
  int32_t mogoTotal = 0;
  float redScore = 0.0;
  float blueScore = 0.0;
  float mogoScore = 0.0;

  for (int i = 0; i < SAMPLE_WINDOW; i++) {
    if (aiSamples[i].type == redRing) {
      redTotal++;
      redScore += aiSamples[i].score;
    }
    else if (aiSamples[i].type == blueRing) {
      blueTotal++;
      blueScore += aiSamples[i].score;
    }
    else if (aiSamples[i].type == mobileGoal) {
      mogoTotal++;
      mogoScore += aiSamples[i].score;
    }
  }

  aiStats.redTotal = redTotal;
  aiStats.blueTotal = blueTotal;
  aiStats.mogoTotal = mogoTotal;
  aiStats.redScore = redScore / (float) redTotal;
  aiStats.blueScore = blueScore / (float) blueTotal;
  aiStats.mogoScore = mogoScore / (float) mogoTotal;

}

void hasAIObjCallback()
{
  AIVision8.takeSnapshot(aivision::ALL_AIOBJS);
  // Check to see if an AI Classification exists in this snapshot.
  int id, x, y, w, h;
  int score;

  if (AIVision8.objectCount > 0) {

    id = AIVision8.objects[0].id;
    x = AIVision8.objects[0].centerX;
    y = AIVision8.objects[0].centerY;
    w = AIVision8.objects[0].width;
    h = AIVision8.objects[0].height;
    score = AIVision8.objects[0].score;
    aiObj.count = AIVision8.objectCount;

    // Determine which AI Classification is detected.
    if (AIVision8.objects[0].id == mobileGoal) {
      // Conditional based on finding a MOGO

      aiObj.bFound = false;

      if (score > 95) {
        aiObj.x = x;
        aiObj.y = y;
        aiObj.height = h;
        aiObj.width = w;

        DetectObject(&aiObj, bMogoDetectHeight, MOGO_DETECTSIZE);

        int diagSize = int(sqrtf(aiObj.width * aiObj.height) + 0.5);
      }
      
      if (bIsSampling && !bAISamplingQueued) {
        aiSamples[curAISample].type = mobileGoal;
        aiSamples[curAISample].score = score;
      }                 
      if (aiLastObjectType != mobileGoal) {
        aiLastObjectType = mobileGoal;
        printf("mogo %d\n", aiObj.count);
      }
    } else if (AIVision8.objects[0].id == blueRing) {

      float fRingWidth = (float) w;
      // assume pixels are linear angle for now
      float fRingAngle = (x - CameraHRES / 2.0) *  CameraHFOV / CameraHRES;
      aiObj.angle = fRingAngle;
      float fMinDistMM = BlueRignWidthMM / sin(CameraHFOV * (2.0 * M_PI / 360.0));
      float fEstDistMM = BlueRignWidthMM / sin(CameraHFOV * (fRingWidth / CameraHRES) * (2.0 * M_PI / 360.0)) ;
      fEstDistMM = sqrtf(fEstDistMM * fEstDistMM - CameraHeightMM * CameraHeightMM);
      aiObj.distMM = (int) fEstDistMM;
      aiObj.bCentered = (!ISLEFT(aiObj.x) && !ISRIGHT(aiObj.x)) ? true : false;
      int ringDiagSize = int(sqrtf(w * h) + 0.5);
      aiObj.bFound = (ringDiagSize > BLUERING_DETECTSIZE) ? true : false;

      // Conditional based on finding a blueRing.
      if (bIsSampling && !bAISamplingQueued) {
        aiSamples[curAISample].type = blueRing;
        aiSamples[curAISample].score = score;
      }     
      if (aiLastObjectType != blueRing) {
        aiLastObjectType = blueRing;
        printf("blue donut %d\n", aiObj.count);
      }
    } else if (AIVision8.objects[0].id == redRing) {
      // Conditional based on finding a blueBall.
      aiObj.bFound = false;
      if (bIsSampling && !bAISamplingQueued) {
        aiSamples[curAISample].type = redRing;
        aiSamples[curAISample].score = score;
      }   
      if (aiLastObjectType != redRing) {
        aiLastObjectType = redRing;
        printf("red donut %d\n", aiObj.count);
      }
    } else {
      aiObj.bFound = false;
      aiObj.bCentered = false;
      if (bIsSampling && !bAISamplingQueued) {
        aiSamples[curAISample].type = -1;
        aiSamples[curAISample].score = 0.0;
      }   
      // Else condition will print that an AI Classification is detected but it does not match the above cases.
    }

  } else {
    aiObj.bFound = false;
    if (bIsSampling && !bAISamplingQueued) {
      aiSamples[curAISample].type = -1;
    }   
  }

  if (aiObj.bFound) {
    aiObj.type = id;
    aiObj.height = h;
    aiObj.width = w;
    aiObj.x = x;
    aiObj.y = y;
  }

  if (bIsSampling && !bAISamplingQueued) {
    curAISample++;
    if (curAISample >= SAMPLE_WINDOW) {
      calcAIStats();
      curAISample = 0;
      bAISamplingQueued = true;
    }
  }

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
 
  if (colObj.height > 0) {
    Brain.Screen.print("%d Green Objects Found", colObj.count);
    Brain.Screen.newLine();
    Brain.Screen.clearLine(2, black);
    Brain.Screen.setCursor(Brain.Screen.row(), 1);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("%0.1f | %dmm",
      colObj.angle, colObj.distMM);
    //Brain.Screen.print("%0.1f | %3d,%3d | %3dx%3d\n",
    //  objectAngle, objectX, objectY,
    //  objectW, objectH);
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

  Brain.Screen.clearLine(3, black);
  Brain.Screen.setCursor(Brain.Screen.row(), 1);
  Brain.Screen.setCursor(3, 1);

  if (aiObj.height > 0) {
    Brain.Screen.print("%d AI Objects Found", aiObj.count);
    Brain.Screen.newLine();
    Brain.Screen.clearLine(4, black);
    Brain.Screen.setCursor(Brain.Screen.row(), 1);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("%c %0.1f | %dmm",
      (aiObj.type == mobileGoal) ? "M" : ((aiObj.type == redRing) ? "R" : "B"),
      aiObj.angle, aiObj.distMM);
  } else {
    if (printThis) {
      Brain.Screen.print("No AI Object");
    }
  }

  if (bIsSampling && bSamplingQueued) {
    if (!voPrint(&visionStats)) {
      printf("vision stats: v(%d) c(%d,%d,%d,%d,%0.1f) x(%.2f, %0.2f) y(%.2f, %0.2f) s(%.2f, %0.2f, %d) d(%d,%d,%d), c(%.2f, %0.2f)\n",
        visionStats.totalValid,
        visionStats.red, visionStats.green, visionStats.blue,
        visionStats.hue,
        visionStats.sat,
        visionStats.xMean, visionStats.xVar,
        visionStats.yMean, visionStats.yVar,
        visionStats.sizeMean, visionStats.sizeVar, visionStats.maxSize,
        visionStats.dist1, visionStats.dist2, visionStats.dist3,
        visionStats.countMean, visionStats.countVar);
    }
    bSamplingQueued = false;
  }
  if (bIsSampling && bAISamplingQueued) {
    printf("ai stats: %d/%0.1f red, %d/%0.1f blue, %d/%0.1f mogo\n",
      aiStats.redTotal,
      aiStats.redScore,
      aiStats.blueTotal,
      aiStats.blueScore,
      aiStats.mogoTotal,
      aiStats.mogoScore
      );
    bAISamplingQueued = false;
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
  bool bTrackBlueRing = true;

  EDRIVESTATE curState = ALLSTOP;
  EDRIVESTATE nextState;

  int loopCount = 0;
  int startCount = 0;

  int bSelectedFound = false;
  int bSelectedCentered = false;
  float selectedAngle = 0.0;
  float selectedDistMM = 0.0;
  int selectedX = 0;

  while(!bCancelVisionTest) {
    // TODO: Add last known location to direction of turn
    // Put filter on lost object

    if (bTrackBlueRing) {
      bSelectedFound = aiObj.bFound;
      bSelectedCentered = aiObj.bCentered;
      selectedAngle = aiObj.angle;
      selectedDistMM = aiObj.distMM;
      selectedX = aiObj.x;
    } else {
      bSelectedFound = colObj.bFound;
      bSelectedCentered = colObj.bCentered;
      selectedAngle = colObj.angle;
      selectedDistMM = colObj.distMM;
      selectedX = colObj.x;
    }

    if (curState == ALLSTOP) {
      if (bSelectedFound) nextState = TRACKING;
      else nextState = SEARCHING_START;

    } else if (curState == SEARCHING_START) {
      Drivetrain.setTurnVelocity(50.0, vex::percent);    
      Drivetrain.turnFor((bWasLeft) ? -360.0 : 360.0, vex::degrees, false);
      startCount = loopCount;
      nextState = SEARCHING;

    } else if (curState == SEARCHING) {
      if (bSelectedFound) {
        Drivetrain.stop(coast);
        nextState = TRACKING;
        // BUGBUG: Never completes
        //      } else if (Drivetrain.isDone()) { 
      } else if (loopCount - startCount > 200) {
        Drivetrain.stop(coast);
        nextState = ALLSTOP;
      }

    } else if (curState == TRACKING) {
      float driveSpeed = 0.0;
      float turnSpeed = 0.0;
      if (bSelectedFound) {
        if (bSelectedCentered) {
          if (selectedAngle < -1.0) {
            turnSpeed = -5.0;
            bWasLeft = true; 
          } else if (selectedAngle > 1.0) {
            turnSpeed = 5.0;
            bWasLeft = false; 
          }
          int calcDistMM = 0;
          if (dsDistance > 0) {
            if (dsDistance < 250) calcDistMM = dsDistance;
            else calcDistMM = (dsDistance + selectedDistMM) / 2; 
          } else {
            calcDistMM = selectedDistMM;
          }
          if (calcDistMM > 500) {
            driveSpeed = 50.0;
          }else if (calcDistMM > 250) {
            driveSpeed = 25.0;
          } else if (calcDistMM < 200) {
            driveSpeed = -5.0;
          }
        } else {
          if (ISLEFT(selectedX)) {
            turnSpeed = -15.0;    
            bWasLeft = true;
          } else if (ISRIGHT(selectedX)) {
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
  printf("Camera mount angle: %0.1fdeg\n", CameraNearAngle);

  // uint8_t red, green, blue;
  // AIVision8.getWhiteBalanceValues(&red, &green, &blue);
  AIVision8.startAwb();
  printf("Vision Thread: AWB complete\n");
  AIVision8.colorDetection(true);
  AIVision8.modelDetection(true);

#ifdef COLMOGO
  iColRed = (int) AIVision8__YELLOWMOGO.red;
  iColGreen = (int) AIVision8__YELLOWMOGO.green;
  iColBlue = (int) AIVision8__YELLOWMOGO.blue;
  iColHue = (int) AIVision8__YELLOWMOGO.hangle;
  fColSat = AIVision8__YELLOWMOGO.hdsat;
 
  AIVision8__TEMP =
    new aivision::colordesc(
      1,
      iColRed,
      iColGreen,
      iColBlue,
      iColHue,
      fColSat);

  colPysicalDimMM = MOGOWidthMM;
  bColDetecHeight = bMogoDetectHeight;
  colDetectSize = MOGO_DETECTSIZE;

#endif

#ifdef COLGREENTRIBALL
  iColRed = (int) AIVision8__GRNTRIBALL.red;
  iColGreen = (int) AIVision8__GRNTRIBALL.green;
  iColBlue = (int) AIVision8__GRNTRIBALL.blue;
  iColHue = (int) AIVision8__GRNTRIBALL.hangle;
  fColSat = AIVision8__GRNTRIBALL.hdsat;
 
  AIVision8__TEMP =
    new aivision::colordesc(
      1,
      iColRed,
      iColGreen,
      iColBlue,
      (float) iColHue,
      fColSat);

  colPysicalDimMM = TriballHeightMM;
  bColDetecHeight = bTriballDetectHeight;
  colDetectSize = TRIBALL_DETECTSIZE;

#endif

#ifdef COLREDDONUT
  fGreenHue = AIVision8__REDDONUT.hangle;
  fGreenSat = AIVision8__REDDONUT.hdsat;
 
  AIVision8__TEMP =
    new aivision::colordesc(
      AIVision8__REDDONUT.id,
      AIVision8__REDDONUT.red,
      AIVision8__REDDONUT.green,
      AIVision8__REDDONUT.blue,
      fGreenHue,
      fGreenSat);

  colPysicalDimMM = RedRignWidthMM;
  bColDetecHeight = bRedRingDetectHeight;
  colDetectSize = REDRING_DETECTSIZE;

#endif

  AIVision8.set(*AIVision8__TEMP);

  checkGreen(hasGreenCallback);
  checkAIObj(hasAIObjCallback);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    //uint32_t startTime = vex::timer::system();
    // Takes about 4-6ms
    checkGreen.broadcastAndWait();
    checkAIObj.broadcastAndWait();
    //uint32_t endTime = vex::timer::system();
    //printf("time: %lu\n", endTime - startTime);
    this_thread::sleep_for(20);
  }
}

void StartVision()
{
  vex::thread vt = vex::thread(VisionThread);
}

