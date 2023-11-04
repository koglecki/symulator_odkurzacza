#include "RobotController.hpp"
#include <iostream>
#include <cmath>

int main(int argc, char **argv) {
    //CleaningRobot* cr = new CleaningRobot(0.51, 1.7, 3.14159);
    //CleaningRobot* cr = new CleaningRobot(1.79, 1.78, 1.5708);
    CleaningRobot* cr = new CleaningRobot(1.16, -0.51, 0);
    Map* map = new Map();
    RobotController* controller = new RobotController(cr, map);
    map->setMapCorrectionValue(40);
    map->setArenaSize(400, 400);
    
    while (cr->robot->step(cr->getTimeStep()) != -1) {      // g��wna p�tla programu
        std::cout << "mode = " << controller->getMode() << std::endl;

        if (controller->isCleaning())
            cr->calculatePosition(0);                // obliczanie nowej pozycji robota
        else
            cr->calculatePosition(1);

        const float* lidarScan = cr->getLidarScan();    // pobranie aktualnych danych z lidara
        std::cout << *(lidarScan + 199) << std::endl;
 
        if (!controller->isCleaning()) {
            if (map->isFirstTurn() || map->isMapping() || map->isMapOpened()) {
                cr->refreshDisplay(lidarScan);

                controller->checkMap();                 // sprawdzanie warunk�w otwarcia i zamkni�cia mapy
                controller->checkObstacles(lidarScan);  // wykrywanie przeszk�d
                controller->chooseMode(lidarScan);      // wyb�r trybu pracy robota
            }
            else if (!map->isFirstTurn() && !map->isMapping() && !map->isMapOpened() && cr->getPoseSensor()[0] - cr->getPrevPoseSensor()[0] == 0 && cr->getPoseSensor()[1] - cr->getPrevPoseSensor()[1] == 0) {
                controller->startCleaning();
                map->createMap(cr->getPosition()[0], cr->getPosition()[1]);
                cr->drawMap(map->getMap());
            }
            else if (!map->isFirstTurn() && !map->isMapping() && !map->isMapOpened())
                controller->chooseMode(lidarScan);           
        }
        else if (!map->areNeighbourCellsOccupied(cr->getPosition()[0], cr->getPosition()[1]) && controller->isGridFinding())
            controller->planPath();
        else if (map->areNeighbourCellsOccupied(cr->getPosition()[0], cr->getPosition()[1]) && controller->isGridFinding())
            controller->planPathToPoint();    
        else if (controller->isCleaning() && controller->isObstacleAvoidance()) {
            //controller->checkObs();
            controller->checkObstacles2(lidarScan);
            controller->chooseMode(lidarScan);
        }
        else {
            if (!controller->checkObstacleTransform())
                controller->Lidar(lidarScan);
            controller->chooseMode(lidarScan);
        }
        
        cr->refreshSensorValues();
    }
    delete cr, map, controller;
    return 0;
}
//0.01m = 1 piksel