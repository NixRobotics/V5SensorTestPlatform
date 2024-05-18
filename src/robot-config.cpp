#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftRearDriveSmart = motor(PORT2, ratio18_1, false);
motor RightRearDriveSmart = motor(PORT1, ratio18_1, true);
motor LeftFrontDriveSmart = motor(PORT4, ratio18_1, false);
motor RightFrontDriveSmart = motor(PORT3, ratio18_1, true);
motor_group LeftMotorGroup = motor_group(LeftRearDriveSmart, LeftFrontDriveSmart);
motor_group RightMotorGroup = motor_group(RightRearDriveSmart, RightFrontDriveSmart);
inertial InertialSensor = inertial(PORT6);
distance DistanceSensor = distance(PORT7);
// gyro TurnGyroSmart = gyro(Brain.ThreeWirePort.D);
smartdrive Drivetrain = smartdrive(LeftMotorGroup, RightMotorGroup, InertialSensor, 6.283185, 14.25, 14.5, inches, 1);
// drivetrain Drivetrain = drivetrain(LeftMotorGroup, RightMotorGroup, 6.283185, 14.75, 14.5, inches, 1);

// https://www.vexforum.com/t/vexcode-motor-groups-and-drivetrain-example/69161
// https://www.vexforum.com/t/how-do-i-set-up-the-heading-for-an-inertial-sensor/84238

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */

void vexcodeInit( void ) {
    printf("vexcodeInit++\n");
    Brain.Screen.print("Device initialization...");
    Brain.Screen.setCursor(2, 1);
    // calibrate the drivetrain gyro
    wait(200, msec);
    InertialSensor.calibrate(2);
    Brain.Screen.print("Calibrating Gyro for Drivetrain");
    // wait for the gyro calibration process to finish
    while (InertialSensor.isCalibrating()) {
        printf(".");
        wait(25, msec);
    }
    // reset the screen now that the calibration is complete
    printf("\n");
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    wait(50, msec);
    Brain.Screen.clearScreen();
    printf("vexcodeInit--\n");
}