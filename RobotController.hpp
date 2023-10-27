#ifndef RobotController_hpp
#define RobotController_hpp
#include "Map.hpp"
#include "CleaningRobot.hpp"

class RobotController {
private:
    int mode = 3;       //tryb pracy robota
    CleaningRobot* robot;
    Map* map;
    bool cleaning = false;
    double startAngle = 0;
    double distance = 0;
    double targetAngle = 0;
    bool xd = true;
    bool xd2 = true;
    double pointX = 0;
    double pointY = 0;
    double startCoord = 0;

public:
    RobotController(CleaningRobot* cr, Map* m);

    void setMode(int m);

    int getMode();

    bool isCleaning();

    void startCleaning();

    void checkMap();

    void checkObstacles(const float* rangeImage);

    void chooseMode(const float* rangeImage);

    double dist(double x, double y);

    double distMax(double x, double y);
};
#endif