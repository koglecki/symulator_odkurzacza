#ifndef CleaningRobot_hpp
#define CleaningRobot_hpp

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Lidar.hpp>
#include <webots/Display.hpp>
#include <webots/Pen.hpp>

using namespace webots;

const double pi = 3.14159265358979323846;

class CleaningRobot {
private:
    const double wheelRadius = 0.04;
    const double wheelbase = 0.26;
    const double robotRadius = 0.169;
    int timeStep;
    double poseSensor[2] = { 0, 0 };         //k¹t przebyty przez lewe i prawe ko³o wed³ug czujników (radiany)
    double prevPoseSensor[2] = { 0, 0 };     //poprzedni pomiar k¹ta
    double position[3] = { 0, 0, 0 };      //aktualna pozycja robota (x,y,theta)                  
    int displayWidth;
    int displayHeight;

    void clearRobotDisplay();
    void setRobot();

public:
    Robot* robot = new Robot();
    Motor* right_motor = robot->getMotor("right_motor");
    Motor* left_motor = robot->getMotor("left_motor");
    PositionSensor* right_position = robot->getPositionSensor("right_position");
    PositionSensor* left_position = robot->getPositionSensor("left_position");
    Lidar* lidar = robot->getLidar("lidar");
    Display* display = robot->getDisplay("display");
    Pen* pen = robot->getPen("pen");

    CleaningRobot();

    CleaningRobot(double x, double y, double theta);

    double* getPosition();

    void drawRobot();

    double* getPrevPoseSensor();

    double* getPoseSensor();

    double getLidarRange();

    void disablePointCloud();

    int getTimeStep();

    const float* getLidarScan();

    void setDriveParameters(double leftVoltage, double rightVoltage);

    void calculatePosition(bool clearDisp);

    void refreshDisplay(const float* rangeImage);

    void refreshSensorValues();

    double* calculatePoint(double distance, int i);

    bool rotateRobotToAngle(double startAngle, double goalAngle);

    bool rotateRobot(double startAngle, double angle);

    void driveRobot(double voltage);

    double calculateDistance();

    bool stopRobot();

    void clearDisplay();

    void drawMap(std::vector <std::vector<bool>> map);

    ~CleaningRobot();
};
#endif