#include "RobotController.hpp"
#include <iostream>
#include <cmath>

int main(int argc, char **argv) {
    CleaningRobot* cr = new CleaningRobot(0.51, 1.7, 3.14159);
    //CleaningRobot* cr = new CleaningRobot(1.79, 1.78, 1.5708);
    //CleaningRobot* cr = new CleaningRobot(1.16, -0.51, 0);
    Map* map = new Map();
    RobotController* controller = new RobotController(cr, map);
    map->setMapCorrectionValue(40);
    map->setArenaSize(500, 600);
    
    while (cr->robot->step(cr->getTimeStep()) != -1) {      // g³ówna pêtla programu
        std::cout << "mode = " << controller->getMode() << std::endl;

        if (controller->isCleaning())
            cr->calculatePosition(0);                // obliczanie nowej pozycji robota
        else
            cr->calculatePosition(1);

        const float* lidarScan = cr->getLidarScan();    // pobranie aktualnych danych z lidara

        if (!controller->isCleaning()) {    // mapowanie
            if (map->isFirstTurn() || map->isMapping() || map->isMapOpened()) {
                cr->refreshDisplay(lidarScan);

                controller->checkMap();                 // sprawdzanie warunków otwarcia i zamkniêcia mapy
                controller->checkObstacles(lidarScan);  // wykrywanie przeszkód
                controller->chooseMode(lidarScan);      // wybór trybu pracy robota
            }
            else if (!map->isFirstTurn() && !map->isMapping() && !map->isMapOpened() && cr->getPoseSensor()[0] - cr->getPrevPoseSensor()[0] == 0 && cr->getPoseSensor()[1] - cr->getPrevPoseSensor()[1] == 0) {
                controller->startCleaning();            // je¿eli mapowanie ukoñczone
                map->createMap(cr->getPosition()[0], cr->getPosition()[1], true);
                cr->drawMap(map->getMap());
            }
            else if (!map->isFirstTurn() && !map->isMapping() && !map->isMapOpened())
                controller->chooseMode(lidarScan);           
        }
        else if (!map->areNeighbourCellsOccupied(cr->getPosition()[0], cr->getPosition()[1]) && controller->isGridFinding())    // je¿eli s¹ nieodwiedzone komórki dooko³a
            controller->planPath();
        else if (map->areNeighbourCellsOccupied(cr->getPosition()[0], cr->getPosition()[1]) && controller->isGridFinding())     // je¿eli trzeba dojechaæ do nieodwiedzonej komórki
            controller->planPathToPoint();    
        else if (controller->isCleaning() && controller->isObstacleAvoidance()) {   // omijanie i skanowanie przeszkody
            controller->checkObs();
            controller->checkObstacles2(lidarScan);
            controller->chooseMode(lidarScan);
            if (!controller->xdd() && !map->isObstacling() && !map->isObsOpened() && cr->getPoseSensor()[0] - cr->getPrevPoseSensor()[0] == 0 && cr->getPoseSensor()[1] - cr->getPrevPoseSensor()[1] == 0) {
                controller->setObstacleAvoidance(false);
                map->createMap(cr->getPosition()[0], cr->getPosition()[1], false);
                map->printGrid();
            }
        }
        else {      // obje¿d¿anie pomieszczenia
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