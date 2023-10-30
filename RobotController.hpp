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
    double pointX = 1;  // wspó³rzêdne rzeczywiste do których udaje siê robot
    double pointY = 1;
    double startCoord = 0;
    bool gridFinding = false;
    std::vector <std::vector<double>> path;
    int pathIterator = 0;

public:
    RobotController(CleaningRobot* cr, Map* m);

    void setMode(int m);

    int getMode();

    bool isCleaning();

    bool isGridFinding();

    bool isRoomClean(std::vector <std::vector<int>> grid);

    void startCleaning();

    void checkMap();

    void findWayToStart(int& currentGridX, int& currentGridY, double& currentX, double& currentY, std::vector <std::vector<int>>& localGrid);

    void checkObstacles(const float* rangeImage);

    void chooseMode(const float* rangeImage);

    double distMax(double x, double y);

    void planPath();

    void optimizePath();

    int chooseWay(bool* equalValues, int currentGridX, int currentGridY, double currentX, double currentY);

    void convertCoords(double &x, double &y);

    std::vector <std::vector<int>> findUnvisitedGrids(std::vector <std::vector<int>> grid);

    std::vector <std::vector<int>> createLocalWavePropagation(std::vector <std::vector<int>> grid);

    void planPathToPoint();

    int* getGoalPoint(std::vector <std::vector<int>> localGrid);

    bool chooseNext(int& currentGridX, int& currentGridY, double& currentX, double& currentY, std::vector <std::vector<int>>& localGrid);
};
#endif