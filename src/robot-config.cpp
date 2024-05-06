#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

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