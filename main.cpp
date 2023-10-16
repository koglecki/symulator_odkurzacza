#include "CleaningRobot.hpp"
#include "Map.hpp"
#include <iostream>
#include <cmath>

class RobotController {
private:
    int mode = 3;       //tryb pracy robota
    CleaningRobot* robot;
    Map* map;
    double startAngle = 0;
    double distance = 0;
    double targetAngle = 0;

public:
    RobotController(CleaningRobot* cr, Map* m) {
        robot = cr;
        map = m;
    }
    void setMode(int m) {
        mode = m;
    }

    int getMode() {
        return mode;
    }

    void checkMap() {       //!!!!!!!!!!! to te� potem mo�na zmieni�
        if (!map->isFirstTurn() && (abs(map->getMapClosurePosition()[0] - robot->getPosition()[0]) > 1 || abs(map->getMapClosurePosition()[1] - robot->getPosition()[1] > 1)))
            map->openMap();            //mo�liwo�� zamkni�cia p�tli mapy

        if (map->isMapping() && map->isMapOpened() && (abs(map->getMapClosurePosition()[0] - robot->getPosition()[0]) < 0.2) && (abs(map->getMapClosurePosition()[1] - robot->getPosition()[1]) < 0.2)) {
            map->closeMap();
            map->finishMapping();
            mode = 4;
            robot->clearDisplay();
            //map->printMap();
            
            
        }
    }

    void checkObstacles(const float* rangeImage) {      //!!!!!!!!!!!!!!!!!!!!!!!!!!! to mo�e poprawi�
        bool obstacles = false;     //czy jakie� przeszkody s� w zasi�gu lidara

        if (mode == 2 || mode == 3 || mode == 7 || mode == 8 || mode == 9) {
            if (*(rangeImage + 199) > 0.4 && map->isMapping() && mode != 7 && mode != 8 && mode != 9)
                mode = 4;
            else {
                for (int i = 0; i < 200; i++) {
                    if (*(rangeImage + i) < 1) {
                        obstacles = true;
                        if (mode != 7 && mode != 8 && mode != 9)
                            mode = 3;
                        if (map->isMapping()) {
                            double* xy = robot->calculatePoint(*(rangeImage + i), i);
                            map->insertPoint(xy[0], xy[1]);
                        }
                    }
                    if (*(rangeImage + i) < 0.2) {
                        mode = 4;
                        if (!map->isWallFound())
                            map->beginMapping();
                        map->setWallFound();
                        break;
                    }
                    //std::cout << i << "->" << *(rangeImage + i) << " ";
                }
                //std::cout << std::endl;
            }
        }

        if (mode == 3 && obstacles == false)      //je�eli robot jedzie i nie ma przeszk�d = przy�piesz
            mode = 2;
    }
    
    void chooseMode(const float* rangeImage) {
        bool obstacleInFront = false;
        bool obstacle = false;
        
        switch (mode) {
        //tryb decyzyjny
        case 1: if (map->isMapping() && map->isFirstTurn()) {    //pocz�tek mapowania
                    map->finishFirstTurn();
                    map->setMapClosurePosition(robot->getPosition()[0], robot->getPosition()[1]);     //wsp�rz�dne rozpoczynaj�ce mapowanie
                }
                else if (map->isMapping() && !map->isFirstTurn()) {     //polecenia przy tworzeniu mapy
                    for (int i = 0; i < 200; i++) {
                        if (*(rangeImage + i) < 0.2) {
                            obstacle = true;
                            if (i > 90 && i < 110) {
                                obstacleInFront = true;
                                break;
                            }
                        }
                    }
                    if (obstacleInFront) {    //przy rogu / �cianie -> obr�t o 90 stopni
                        mode = 5;
                        targetAngle = pi / 2;
                    }
                    else if (*(rangeImage + 199) > 0.6 && !obstacle) {    //przy wewn�trznym rogu -> szukanie �ciany na nowo
                        mode = 9;
                        distance = 0;
                    }
                    else if (*(rangeImage + 199) > *(rangeImage + 196) && *(rangeImage + 199) > 0.4 && *(rangeImage + 199) < 0.6 && !obstacle)    //robot skierowany do �ciany -> podje�d�anie bli�ej �ciany
                        mode = 8;
                    else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.45 && *(rangeImage + 199) < 0.6) {     //wi�kszy obr�t do �ciany w przypadku uskoku na �cianie
                        mode = 5;
                        targetAngle = -0.3;
                    }
                    else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.4 && *(rangeImage + 199) <= 0.45) {   //lekki obr�t do �ciany
                        mode = 5;
                        targetAngle = -0.1;
                    }
                    else {
                        mode = 5;     //lekki obr�t od �ciany
                        targetAngle = 0.1;
                    }
                }
                else
                    mode = 4;
                startAngle = robot->getPosition()[2];
                break;
        case 2: robot->driveRobot(5);  //jazda prosto z pe�n� pr�dko�ci�
                break;
        case 3: robot->driveRobot(2);  //jazda prosto z mniejsz� pr�dko�ci�
                break;
        case 4: if (robot->stopRobot())  //zatrzymywanie robota
                    mode = 1;
                break;      
        case 5: if (robot->turnRobot(startAngle, targetAngle))      // obr�t robota o podany k�t
                    mode = 3;
                break;
        case 6: if (robot->turnRobot(startAngle, -pi / 2))
                    mode = 7;
                break;
        case 7: if (*(rangeImage + 199) > 0.6)
                    robot->driveRobot(0.5);
                 else
                    mode = 4;
                break;
        case 8: if (*(rangeImage + 199) < 0.6 && *(rangeImage + 199) > 0.4)       // powolna jazda do pewnej warto�ci lidara
                    robot->driveRobot(0.5);
                 else
                    mode = 4;
                break;
        case 9: if (distance < 0.3) {          // powolna jazda przez 0.3 metra
                    robot->driveRobot(0.5);
                    distance += robot->calculateDistance();
                 }
                else if (robot->stopRobot())
                    mode = 6;
                break;
        }
    }
};

int main(int argc, char **argv) {
    CleaningRobot* cr = new CleaningRobot(0.51, 1.7, 3.14159);
    Map* map = new Map();
    RobotController* controller = new RobotController(cr, map);   
    
    while (cr->robot->step(cr->getTimeStep()) != -1) {      // g��wna p�tla programu
        std::cout << "mode = " << controller->getMode() << std::endl;

        cr->calculatePosition();                // obliczanie nowej pozycji robota
        const float* lidarScan = cr->getLidarScan();    // pobranie aktualnych danych z lidara
        std::cout << *(lidarScan + 199) << std::endl;
 
        cr->refreshDisplay(lidarScan);
            
        controller->checkMap();                 // sprawdzanie warunk�w otwarcia i zamkni�cia mapy
        controller->checkObstacles(lidarScan);  // wykrywanie przeszk�d
        controller->chooseMode(lidarScan);      // wyb�r trybu pracy robota

        cr->refreshSensorValues();

        if (!map->isFirstTurn() && !map->isMapping() && !map->isMapOpened()) {
            map->createMap();
            cr->drawMap(map->getMap());
            
        }
    }
    delete cr, map, controller;
    return 0;
}
//0.01m = 1 piksel