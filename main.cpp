#include "CleaningRobot.hpp"
#include "Map.hpp"
#include <iostream>
#include <cmath>

class RobotController {
private:
    int mode = 2;       //tryb pracy robota
    CleaningRobot* robot;
    Map* map;
    double startAngle = 0;
    double d = 0;
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

    void checkMap() {
        if (!map->isFirstTurn() && (abs(map->getMapClosurePosition()[0] - robot->getPosition()[0]) > 1 || abs(map->getMapClosurePosition()[1] - robot->getPosition()[1] > 1)))
            map->openMap();            //mo¿liwoœæ zamkniêcia pêtli mapy

        if (map->isMapOpened() && (abs(map->getMapClosurePosition()[0] - robot->getPosition()[0]) < 0.2) && (abs(map->getMapClosurePosition()[1] - robot->getPosition()[1]) < 0.2)) {
            map->closeMap();
            map->finishMapping();
            setMode(3);
            map->printMap();
        }
    }

    void checkObstacles(const float* rangeImage) {
        bool obstacles = false;     //czy jakieœ przeszkody s¹ w zasiêgu lidara

        if (getMode() == 1 || getMode() == 2 || getMode() == 11 || getMode() == 12 || getMode() == 13) {
            if (*(rangeImage + 199) > 0.4 && map->isMapping() && getMode() != 11 && getMode() != 12 && getMode() != 13)
                setMode(3);
            else {
                for (int i = 0; i < 200; i++) {
                    if (*(rangeImage + i) < 1) {
                        obstacles = true;
                        if (getMode() != 11 && getMode() != 12 && getMode() != 13)
                            setMode(2);
                        if (map->isMapping()) {
                            double* xy = robot->calculatePoint(*(rangeImage + i), i);
                            map->insertPoint(xy[0], xy[1]);
                        }
                    }
                    if (*(rangeImage + i) < 0.2) {
                        setMode(3);
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

        if (getMode() == 2 && obstacles == false)      //je¿eli robot jedzie i nie ma przeszkód = przyœpiesz
            setMode(1);
    }
    
    void chooseMode(const float* rangeImage) {
        bool obstacleInFront = false;
        bool obstacle = false;
        
        switch (mode) {
        case 1: robot->driveRobot(5);  //jazda prosto z pe³n¹ prêdkoœci¹
                break;
        case 2: robot->driveRobot(2);  //jazda prosto z mniejsz¹ prêdkoœci¹
                break;
        case 3: if (robot->stopRobot())  //zatrzymywanie robota
                    mode = 4;
                break;
        //tryb decyzyjny
        case 4: if (map->isMapping() && map->isFirstTurn()) {    //pocz¹tek mapowania
                    map->finishFirstTurn();
                    map->setMapClosurePosition(robot->getPosition()[0], robot->getPosition()[1]);     //wspó³rzêdne rozpoczynaj¹ce mapowanie
                    mode = 4;
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
                    if (obstacleInFront) {    //przy rogu / œcianie -> obrót o 90 stopni
                        mode = 6;
                        targetAngle = pi / 2;
                    }
                    else if (*(rangeImage + 199) > 0.6 && !obstacle) {    //przy wewnêtrznym rogu -> szukanie œciany na nowo
                        mode = 11;
                        distance = 0;
                    }
                    else if (*(rangeImage + 199) > *(rangeImage + 196) && *(rangeImage + 199) > 0.4 && *(rangeImage + 199) < 0.6 && !obstacle)    //robot skierowany do œciany -> podje¿d¿anie bli¿ej œciany
                        mode = 13;
                    else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.45 && *(rangeImage + 199) < 0.6) {     //wiêkszy obrót do œciany w przypadku uskoku na œcianie
                        mode = 6;
                        targetAngle = -0.3;
                    }
                    else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.4 && *(rangeImage + 199) <= 0.45) {   //lekki obrót do œciany
                        mode = 6;
                        targetAngle = -0.1;
                    }
                    else {
                        mode = 6;     //lekki obrót od œciany
                        targetAngle = 0.1;
                    }
                }
                else
                    setMode(3);
                startAngle = robot->getPosition()[2];
                break;
        case 6: if (robot->turnRobot(startAngle, targetAngle))      // obrót robota o podany k¹t
                    mode = 2;
                break;
        case 10: if (robot->turnRobot(startAngle, -pi / 2))
                    mode = 12;
                break;
        case 12: if (*(rangeImage + 199) > 0.6)
                    robot->driveRobot(0.5);
                 else
                    mode = 3;
                break;
        case 13: if (*(rangeImage + 199) < 0.6 && *(rangeImage + 199) > 0.4)       // powolna jazda do pewnej wartoœci lidara
                    robot->driveRobot(0.5);
                 else
                    mode = 3;
                break;
        case 11: if (distance < 0.3) {
                    robot->driveRobot(0.5);
                    distance += robot->calculateDistance();
                 }
                else if (robot->stopRobot())
                    mode = 10;
                break;
        }
    }
};

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

        cr->refreshSensorValues();
    }
    delete cr, map, controller;
    return 0;
}
//0.01m = 1 piksel