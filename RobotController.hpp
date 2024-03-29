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
    bool firstObstacleRotation = true;
    bool obstacleAvoidance = false;
    double startAngle = 0;
    double distance = 0;
    double targetAngle = 0;
    double pointX = 1;  // wsp�rz�dne rzeczywiste do kt�rych udaje si� robot
    double pointY = 1;
    double startCoord = 0;
    bool gridFinding = false;
    std::vector <std::vector<double>> path;
    int pathIterator = 0;
    double cond1 = 0;
    double cond2 = 0;
    int distanceTraveled = 0;
    int totalRotates = 0;
    bool extraPathConditions = true;

    void wallFollowing(const float* rangeImage);

    void obstacleFollowing(const float* rangeImage);

    void obstaclesWhenRobotIsStopped(bool& obstacleInFront, bool& obstacle, const float* rangeImage);

public:
    RobotController(CleaningRobot* cr, Map* m);

    void setMode(int m);

    void finishObstacleAvoidance();

    void setPathConditions(bool c);

    bool isFirstObstacleRotation();

    int getMode();

    int getDistanceTraveled();

    int getTotalRotates();

    void checkObstacleCompletion();

    bool isCleaning();

    bool isObstacleAvoidance();

    void setObstacleAvoidance(bool obs);

    bool isObstacleOnLidar(const float* rangeImage);

    bool checkObstacleTransform();

    void checkUnexpectedObstacles(const float* rangeImage);

    bool isGridFinding();

    bool isRoomClean(std::vector <std::vector<int>> grid);

    void startCleaning();

    void checkMap();

    void findWayToStart(int& currentGridX, int& currentGridY, double& currentX, double& currentY, std::vector <std::vector<int>>& localGrid);

    void checkObstacles(const float* rangeImage);

    void checkObstacle(const float* rangeImage);

    void chooseMode(const float* rangeImage);

    double getTargetAngle(double x, double y);

    void planPath();

    void optimizePath();

    void occupyVisitedCells();

    int considerPathConditions(bool* equalValues, int currentGridX, int currentGridY, double currentX, double currentY);

    void convertCoords(double &x, double &y);

    std::vector <std::vector<int>> findUnvisitedGrids(std::vector <std::vector<int>> grid);

    std::vector <std::vector<int>> createLocalWavePropagation(std::vector <std::vector<int>> grid);

    void planPathToPoint();

    int* getGoalPoint(std::vector <std::vector<int>> localGrid);

    bool chooseNextPoint(int& currentGridX, int& currentGridY, double& currentX, double& currentY, std::vector <std::vector<int>>& localGrid);
};
#endif