using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LeftRearDriveSmart;
extern motor RightRearDriveSmart;
extern motor LeftFrontDriveSmart;
extern motor RightFrontDriveSmart;
extern smartdrive Drivetrain;
// extern drivetrain Drivetrain;
extern inertial InertialSensor;
extern distance DistanceSensor;
extern vision::signature Vision8__GREENBOX1;
extern vision Vision8;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );