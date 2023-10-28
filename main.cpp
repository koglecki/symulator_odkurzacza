#include "RobotController.hpp"
#include <iostream>
#include <cmath>

int main(int argc, char **argv) {
    //CleaningRobot* cr = new CleaningRobot(0.51, 1.7, 3.14159);
    CleaningRobot* cr = new CleaningRobot();
    Map* map = new Map();
    RobotController* controller = new RobotController(cr, map);   
    
    while (cr->robot->step(cr->getTimeStep()) != -1) {      // g��wna p�tla programu
        std::cout << "mode = " << controller->getMode() << std::endl;

        cr->calculatePosition();                // obliczanie nowej pozycji robota
        const float* lidarScan = cr->getLidarScan();    // pobranie aktualnych danych z lidara
        std::cout << *(lidarScan + 199) << std::endl;
 
        if (!controller->isCleaning()) {
            cr->refreshDisplay(lidarScan);

            controller->checkMap();                 // sprawdzanie warunk�w otwarcia i zamkni�cia mapy
            controller->checkObstacles(lidarScan);  // wykrywanie przeszk�d
            controller->chooseMode(lidarScan);      // wyb�r trybu pracy robota

            if (!map->isFirstTurn() && !map->isMapping() && !map->isMapOpened() && cr->getPoseSensor()[0] - cr->getPrevPoseSensor()[0] == 0 && cr->getPoseSensor()[1] - cr->getPrevPoseSensor()[1] == 0) {
                map->createMap(cr->getPosition()[0], cr->getPosition()[1]);
                cr->drawMap(map->getMap(), map->getGrid());
                controller->startCleaning();
                controller->planPath();
            }   // mo�e bool pathGenerated??
        }
        else 
            controller->chooseMode(lidarScan);
        
        cr->refreshSensorValues();
    }
    delete cr, map, controller;
    return 0;
}
//0.01m = 1 piksel