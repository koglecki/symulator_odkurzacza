#include "RobotController.hpp"
#include <iostream>
#include <cmath>

int main(int argc, char **argv) {
    //CleaningRobot* cr = new CleaningRobot(0.51, 1.7, 3.14159);
    CleaningRobot* cr = new CleaningRobot(1.64, -1.03, 0);
    //CleaningRobot* cr = new CleaningRobot(1.16, -0.51, 0);
    Map* map = new Map();
    RobotController* controller = new RobotController(cr, map);
    map->setMapCorrectionValue(20);
    map->setArenaSize(500, 400);
    controller->setPathConditions(true);
    
    while (cr->robot->step(cr->getTimeStep()) != -1) {      // g³ówna pêtla programu
        //std::cout << "mode = " << controller->getMode() << std::endl;

        if (controller->isCleaning())
            cr->calculatePosition(0);                // obliczanie nowej pozycji robota
        else
            cr->calculatePosition(1);

        const float* lidarScan = cr->getLidarScan();    // pobranie aktualnych danych z lidara


        if (!controller->isCleaning()) {    // mapowanie         
            if (!map->isFirstRotation() && !map->isMapping() && !map->isMapOpened()
                && cr->getPoseSensor()[0] - cr->getPrevPoseSensor()[0] == 0
                && cr->getPoseSensor()[1] - cr->getPrevPoseSensor()[1] == 0) // je¿eli mapowanie ukoñczone
            {
                controller->startCleaning();
                map->createMap(cr->getPosition()[0], cr->getPosition()[1], true);
                cr->drawMap(map->getMap());
            }
            else {
                cr->refreshDisplay(lidarScan);
                controller->checkMap();                 // sprawdzanie warunków otwarcia i zamkniêcia mapy
                controller->checkObstacles(lidarScan);  // wykrywanie przeszkód
                controller->chooseMode(lidarScan);      // wybór trybu pracy robota
            }
        }

        else if (controller->isRoomClean(map->getGrid())) {     // czy wszystkie kratki zosta³y odwiedzone przez robota
            std::cout << "Dlugosc sciezki robota: " << controller->getDistanceTraveled() << std::endl;
            std::cout << "Laczna liczba zakretow: " << controller->getTotalRotates() << std::endl;
            break;
        }

        else if (!map->areNeighbourCellsOccupied(cr->getPosition()[0], cr->getPosition()[1]) && controller->isGridFinding())    // je¿eli s¹ nieodwiedzone komórki dooko³a
            controller->planPath();

        else if (map->areNeighbourCellsOccupied(cr->getPosition()[0], cr->getPosition()[1]) && controller->isGridFinding())     // je¿eli trzeba dojechaæ do nieodwiedzonej komórki
            controller->planPathToPoint();  

        else if (controller->isCleaning() && controller->isObstacleAvoidance()) {   // omijanie i skanowanie przeszkody
            if (!controller->isFirstObstacleRotation() && !map->isObstacling() && !map->isObsOpened()
                && cr->getPoseSensor()[0] - cr->getPrevPoseSensor()[0] == 0
                && cr->getPoseSensor()[1] - cr->getPrevPoseSensor()[1] == 0)    // je¿eli obje¿d¿anie przeszkody zakoñczone
            {
                controller->setObstacleAvoidance(false);
                controller->finishObstacleAvoidance();
                map->createMap(cr->getPosition()[0], cr->getPosition()[1], false);
                map->printGrid();
            }
            else {
                controller->checkObstacleCompletion();
                controller->checkObstacle(lidarScan);
                controller->chooseMode(lidarScan);
            }
        }

        else {      // obje¿d¿anie pomieszczenia
            if (!controller->checkObstacleTransform())
                controller->checkUnexpectedObstacles(lidarScan);
            controller->chooseMode(lidarScan);
        }
        
        cr->refreshSensorValues();
    }
    delete cr, map, controller;
    return 0;
}
//0.01m = 1 piksel