#include "RobotController.hpp"
#include <iostream>
#include <cmath>

int main(int argc, char **argv) {
    CleaningRobot* cr = new CleaningRobot(0.51, 1.7, 3.14159);
    Map* map = new Map();
    RobotController* controller = new RobotController(cr, map);   
    
    while (cr->robot->step(cr->getTimeStep()) != -1) {      // g³ówna pêtla programu
        std::cout << "mode = " << controller->getMode() << std::endl;

        cr->calculatePosition();                // obliczanie nowej pozycji robota
        const float* lidarScan = cr->getLidarScan();    // pobranie aktualnych danych z lidara
        std::cout << *(lidarScan + 199) << std::endl;
 
        cr->refreshDisplay(lidarScan);
            
        controller->checkMap();                 // sprawdzanie warunków otwarcia i zamkniêcia mapy
        controller->checkObstacles(lidarScan);  // wykrywanie przeszkód
        controller->chooseMode(lidarScan);      // wybór trybu pracy robota

        if (!map->isFirstTurn() && !map->isMapping() && !map->isMapOpened()) {
            map->createMap();
            cr->drawMap(map->getMap(), map->getGrid());
        }

        cr->refreshSensorValues();
    }
    delete cr, map, controller;
    return 0;
}
//0.01m = 1 piksel