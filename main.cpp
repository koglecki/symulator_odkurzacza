#include "CleaningRobot.hpp"
#include "Map.hpp"
#include <iostream>
#include <cmath>

int main(int argc, char **argv) {
    CleaningRobot* cr = new CleaningRobot();
    Map* map = new Map();
    double startAngle = cr->getPosition()[2];   
    double distance = 0;
    double d;
    
    
    while (cr->robot->step(cr->getTimeStep()) != -1) {      //g��wna p�tla programu
        std::cout << "mode = " << cr->getMode() << std::endl;

        const float* rangeImage = cr->calculatePosition();
        std::cout << *(rangeImage + 199) << std::endl;
        double currentPosition[3] = {cr->getPosition()[0], cr->getPosition()[1], cr->getPosition()[2] };
 
        cr->refreshDisplay(rangeImage);
        
        bool obstacles = false;     //czy jakie� przeszkody s� w zasi�gu lidara
        bool check = false;
        bool check2 = false;
        //std::cout << "mapa size   " << map->getMap().size() << std::endl;
        if (!map->isFirstTurn() && (abs(map->getMapClosurePosition()[0] - currentPosition[0]) > 1 || abs(map->getMapClosurePosition()[1] - currentPosition[1] > 1)))
            map->openMap();            //mo�liwo�� zamkni�cia p�tli mapy

        if (map->isMapOpened() && (abs(map->getMapClosurePosition()[0] - currentPosition[0]) < 0.2) && (abs(map->getMapClosurePosition()[1] - currentPosition[1]) < 0.2)) {
            map->closeMap();
            map->finishMapping();
            cr->setMode(3);
            map->printMap();
            //std::map <double, double> ::iterator it;
            //int ggg = 1;
            //for (it = map->getMap().begin(); it != map->getMap().end(); ++it) {
                //std::cout << ggg << " -> " << it->first << " => " << it->second << std::endl;
                //ggg = ggg + 1;
            //}
        }
        if (cr->getMode() == 1 || cr->getMode() == 2 || cr->getMode() == 11 || cr->getMode() == 12) {
            if (*(rangeImage + 199) > 0.4 && map->isMapping() && cr->getMode() != 11 && cr->getMode() != 12)
                cr->setMode(3);
            else {
                for (int i = 0; i < 200; i++) {
                    if (*(rangeImage + i) < 1) {
                        obstacles = true;
                        if (cr->getMode() != 11 && cr->getMode() != 12)
                            cr->setMode(2);
                        if (map->isMapping()) {
                            double* xy = cr->calculatePoint(*(rangeImage + i), i);
                            map->insertPoint(xy[0], xy[1]);
                        }
                    }
                    if (*(rangeImage + i) < 0.2) {
                        cr->setMode(3);
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
        
        if (cr->getMode() == 2 && obstacles == false)      //je�eli robot jedzie i nie ma przeszk�d = przy�piesz
            cr->setMode(1);

        switch (cr->getMode()) {
        case 1: cr->driveRobot(5);  //jazda prosto z pe�n� pr�dko�ci�
                break;
        case 2: cr->driveRobot(2);  //jazda prosto z mniejsz� pr�dko�ci�
                break;
        case 3: cr->stopRobot();    //zatrzymywanie robota
                break;
        case 4: if (map->isMapping() && map->isFirstTurn())     //pocz�tek mapowania
                    cr->setMode(5);
                else if (map->isMapping() && !map->isFirstTurn()) {     //polecenia przy obje�d�aniu terenu
                    for (int i = 0; i < 200; i++) {
                        if (*(rangeImage + i) < 0.2) {
                            check2 = true;
                            if (i > 90 && i < 110) {
                                check = true;
                                break;
                            }                                   
                        }
                    }
                    if (check)     //przy rogu / �cianie
                        cr->setMode(7);
                    else if (*(rangeImage + 199) > 0.6 && !check2) {    //przy wewn�trznym rogu
                        cr->setMode(11);
                        distance = 0;
                    }
                    else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.45 && *(rangeImage + 199) < 0.6) {      //wi�kszy obr�t w przypadku uskoku na �cianie
                        cr->setMode(9);
                    }
                    else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.4 && *(rangeImage + 199) <= 0.45)    //do �ciany
                        cr->setMode(6);
                    else   //od �ciany    
                        cr->setMode(8);
                    
                }
                else
                    cr->setMode(3);        
                startAngle = cr->getPosition()[2];
                break;
        case 5: map->finishFirstTurn();
                map->setMapClosurePosition(currentPosition[0], currentPosition[1]);     //pocz�tek tworzenia mapy
                if (*(rangeImage + 199) < 0.2 || *(rangeImage + 199) > 0.4)
                    cr->turnRobot(startAngle, pi);
                else
                    cr->stopTurningRobot();
                break; 
        case 6: cr->turnRobot(startAngle, -0.1);
                break;
        case 7: cr->turnRobot(startAngle, pi/2);
                break;
        case 8: cr->turnRobot(startAngle, 0.1);    //obroty robota w r�nych przypadkach
                break;
        case 9: cr->turnRobot(startAngle, -0.3);
                break;
        case 10: cr->turnRobot2(startAngle, -pi/2);
                break;
        
        case 12: if (*(rangeImage + 199) > 0.6)
                    cr->driveRobot(0.5);
                 else
                    cr->stopRobot();
                 break;
        case 11:

            d = cr->driveRobotByDistance(0.5, 0.3 - distance);
            distance += d;
            startAngle = cr->getPosition()[2];
            break;
        }


        cr->refreshSensorValues();
    }
    delete cr, map;
    return 0;
}
//0.01m = 1 piksel