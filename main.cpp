#include "RobotController.hpp"
#include <iostream>
#include <cmath>

int main(int argc, char **argv) {
    CleaningRobot* cr = new CleaningRobot(0.51, 1.7, 3.14159);
    //CleaningRobot* cr = new CleaningRobot(1.79, 1.78, 1.5708);
    Map* map = new Map();
    RobotController* controller = new RobotController(cr, map);   
    
    while (cr->robot->step(cr->getTimeStep()) != -1) {      // g³ówna pêtla programu
        std::cout << "mode = " << controller->getMode() << std::endl;

        cr->calculatePosition();                // obliczanie nowej pozycji robota
        const float* lidarScan = cr->getLidarScan();    // pobranie aktualnych danych z lidara
        std::cout << *(lidarScan + 199) << std::endl;
 
        if (!controller->isCleaning()) {
            cr->refreshDisplay(lidarScan);

            controller->checkMap();                 // sprawdzanie warunków otwarcia i zamkniêcia mapy
            controller->checkObstacles(lidarScan);  // wykrywanie przeszkód
            controller->chooseMode(lidarScan);      // wybór trybu pracy robota

            if (!map->isFirstTurn() && !map->isMapping() && !map->isMapOpened() && cr->getPoseSensor()[0] - cr->getPrevPoseSensor()[0] == 0 && cr->getPoseSensor()[1] - cr->getPrevPoseSensor()[1] == 0) {
                map->createMap(cr->getPosition()[0], cr->getPosition()[1]);
                cr->drawMap(map->getMap(), map->getGrid());
                controller->startCleaning();               
            }   // mo¿e bool pathGenerated??
        }
        else if (!map->areNeighbourCellsOccupied(cr->getPosition()[0], cr->getPosition()[1]) && controller->isGridFinding())
            controller->planPath();
        else if (map->areNeighbourCellsOccupied(cr->getPosition()[0], cr->getPosition()[1]) && controller->isGridFinding())
            controller->planPathToPoint();      
        else 
            controller->chooseMode(lidarScan);
        
        cr->refreshSensorValues();
    }
    delete cr, map, controller;
    return 0;
}
//0.01m = 1 piksel