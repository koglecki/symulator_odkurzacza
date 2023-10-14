#include "CleaningRobot.hpp"
#include "Map.hpp"
#include <iostream>
#include <cmath>

class RobotController {
private:
    int mode = 1;       //tryb pracy robota
    CleaningRobot* robot;
    Map* map;
    double startAngle = 0;
    double d = 0;
    double distance = 0;

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
        bool check = false;
        bool check2 = false;
        
        

        switch (mode) {
        case 1: robot->driveRobot(5);  //jazda prosto z pe³n¹ prêdkoœci¹
            break;
        case 2: robot->driveRobot(2);  //jazda prosto z mniejsz¹ prêdkoœci¹
            break;
        case 3: robot->stopRobot();    //zatrzymywanie robota
            if (robot->getPoseSensor()[0] - robot->getPrevPoseSensor()[0] == 0) {
                if (mode == 11)
                    mode = 10;
                else
                    mode = 4;
            }
            break;
        case 4: if (map->isMapping() && map->isFirstTurn())     //pocz¹tek mapowania
            setMode(5);
              else if (map->isMapping() && !map->isFirstTurn()) {     //polecenia przy obje¿d¿aniu terenu
            for (int i = 0; i < 200; i++) {
                if (*(rangeImage + i) < 0.2) {
                    check2 = true;
                    if (i > 90 && i < 110) {
                        check = true;
                        break;
                    }
                }
            }
            if (check)     //przy rogu / œcianie
                setMode(7);
            else if (*(rangeImage + 199) > 0.6 && !check2) {    //przy wewnêtrznym rogu
                setMode(11);
                distance = 0;
            }
            else if (*(rangeImage + 199) > *(rangeImage + 196) && *(rangeImage + 199) > 0.4 && *(rangeImage + 199) < 0.6 && !check2)    //
                setMode(13);
            else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.45 && *(rangeImage + 199) < 0.6) {      //wiêkszy obrót w przypadku uskoku na œcianie
                setMode(9);
            }
            else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.4 && *(rangeImage + 199) <= 0.45)    //do œciany
                setMode(6);
            else   //od œciany    
                setMode(8);

        }
              else
            setMode(3);
            startAngle = robot->getPosition()[2];
            break;
        case 5: map->finishFirstTurn();
            map->setMapClosurePosition(robot->getPosition()[0], robot->getPosition()[1]);     //pocz¹tek tworzenia mapy
            if (*(rangeImage + 199) < 0.2 || *(rangeImage + 199) > 0.4)
                robot->turnRobot(startAngle, pi);
            else
                robot->stopTurningRobot();
                if (robot->getPoseSensor()[0] - robot->getPrevPoseSensor()[0] == 0)
                    mode = 2;
            break;
        case 6: if (robot->turnRobot(startAngle, -0.1))
                    mode = 2;
                 break;
        case 7: if (robot->turnRobot(startAngle, pi / 2))
                    mode = 2;
            break;
        case 8: if (robot->turnRobot(startAngle, 0.1))      // obroty robota w ró¿nych przypadkach
                    mode = 2;
            break;
        case 9: robot->turnRobot(startAngle, -0.3);
            if (robot->getPoseSensor()[0] - robot->getPrevPoseSensor()[0] == 0)
                mode = 2;
            break;
        case 10: robot->turnRobot2(startAngle, -pi / 2);
            if (robot->getPoseSensor()[0] - robot->getPrevPoseSensor()[0] == 0)
                mode = 12;
            break;

        case 12: if (*(rangeImage + 199) > 0.6)
            robot->driveRobot(0.5);
               else {
            robot->stopRobot();
            if (robot->getPoseSensor()[0] - robot->getPrevPoseSensor()[0] == 0)
                mode = 4;
        }
            break;
        case 13: if (*(rangeImage + 199) < 0.6 && *(rangeImage + 199) > 0.4)
            robot->driveRobot(0.5);
               else {
            robot->stopRobot();
            if (robot->getPoseSensor()[0] - robot->getPrevPoseSensor()[0] == 0)
                mode = 4;
        }
            break;
        case 11:

            d = robot->driveRobotByDistance(0.5, 0.3 - distance);
            distance += d;
            startAngle = robot->getPosition()[2];
            if (distance - d > 0.3)
                mode = 10;
            break;
        }
    }
};

int main(int argc, char **argv) {
    CleaningRobot* cr = new CleaningRobot(0.51, 1.7, 3.14159);
    Map* map = new Map();
    RobotController* controller = new RobotController(cr, map);   
    
    while (cr->robot->step(cr->getTimeStep()) != -1) {      //g³ówna pêtla programu
        std::cout << "mode = " << controller->getMode() << std::endl;

        cr->calculatePosition();
        const float* rangeImage = cr->getLidarScan();
        std::cout << *(rangeImage + 199) << std::endl;
        double currentPosition[3] = {cr->getPosition()[0], cr->getPosition()[1], cr->getPosition()[2] };
 
        cr->refreshDisplay(rangeImage);

        
        
        controller->checkMap();
        controller->checkObstacles(rangeImage);
        controller->chooseMode(rangeImage);


        cr->refreshSensorValues();
    }
    delete cr, map, controller;
    return 0;
}
//0.01m = 1 piksel