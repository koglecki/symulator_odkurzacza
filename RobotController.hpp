#ifndef RobotController_hpp
#define RobotController_hpp
#include "Map.hpp"
#include "CleaningRobot.hpp"

class RobotController {
private:
    int mode = 3;       //tryb pracy robota
    CleaningRobot* robot;
    Map* map;
    double startAngle = 0;
    double distance = 0;
    double targetAngle = 0;

public:
    RobotController(CleaningRobot* cr, Map* m);

    void setMode(int m);

    int getMode();

    void checkMap();

    void checkObstacles(const float* rangeImage);

    void chooseMode(const float* rangeImage);
};
#endif