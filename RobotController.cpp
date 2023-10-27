#include "RobotController.hpp"

    RobotController::RobotController(CleaningRobot* cr, Map* m) {
        robot = cr;
        map = m;
    }
    void RobotController::setMode(int m) {
        mode = m;
    }

    int RobotController::getMode() {
        return mode;
    }

    bool RobotController::isCleaning() {
        return cleaning;
    }

    void RobotController::startCleaning() {
        cleaning = true;
    }

    void RobotController::checkMap() {       //!!!!!!!!!!! to te¿ potem mo¿na zmieniæ
        if (!map->isFirstTurn() && (abs(map->getMapClosurePosition()[0] - robot->getPosition()[0]) > 1 || abs(map->getMapClosurePosition()[1] - robot->getPosition()[1] > 1)))
            map->openMap();            //mo¿liwoœæ zamkniêcia pêtli mapy

        if (map->isMapping() && map->isMapOpened() && (abs(map->getMapClosurePosition()[0] - robot->getPosition()[0]) < 0.2) && (abs(map->getMapClosurePosition()[1] - robot->getPosition()[1]) < 0.2)) {
            map->closeMap();
            map->finishMapping();
            mode = 4;
            robot->clearDisplay();
            //map->printMap();

        }
    }

    void RobotController::checkObstacles(const float* rangeImage) {      //!!!!!!!!!!!!!!!!!!!!!!!!!!! to mo¿e poprawiæ
        bool obstacles = false;     //czy jakieœ przeszkody s¹ w zasiêgu lidara

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

        if (mode == 3 && obstacles == false)      //je¿eli robot jedzie i nie ma przeszkód = przyœpiesz
            mode = 2;
    }

    double RobotController::dist(double x, double y) {
        bool isY = true;
        double angle = robot->getPosition()[2];
        if (abs(x - robot->getPosition()[0]) < abs(y - robot->getPosition()[1]))
            isY = false;
        if (isY && abs(y - robot->getPosition()[1]) > 0.01) {
            if (y < robot->getPosition()[1])
                angle = 3 * pi / 2;
            else
                angle = pi / 2;
        }
        else if (!isY && abs(x - robot->getPosition()[0]) > 0.01) {
            if (x < robot->getPosition()[0])
                angle = pi;
            else
                angle = 0;
        }
        
        return angle;
    }

    double RobotController::distMax(double x, double y) {
        bool isY = true;
        double angle = robot->getPosition()[2];
        if (abs(x - robot->getPosition()[0]) > abs(y - robot->getPosition()[1]))
            isY = false;
        if (isY && abs(y - robot->getPosition()[1]) > 0.01) {
            if (y < robot->getPosition()[1])
                angle = 3 * pi / 2;
            else
                angle = pi / 2;
        }
        else if (!isY && abs(x - robot->getPosition()[0]) > 0.01) {
            if (x < robot->getPosition()[0])
                angle = pi;
            else
                angle = 0;
        }

        return angle;
    }

    void RobotController::chooseMode(const float* rangeImage) {
        bool obstacleInFront = false;
        bool obstacle = false;
        
        double goalX = 0;
        double goalY = 0;

        switch (mode) {
            //tryb decyzyjny
        case 1: if (map->isMapping() && map->isFirstTurn()) {    //pocz¹tek mapowania
                    map->finishFirstTurn();
                    map->setMapClosurePosition(robot->getPosition()[0], robot->getPosition()[1]);     //wspó³rzêdne rozpoczynaj¹ce mapowanie
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
                        mode = 5;
                        targetAngle = pi / 2;
                    }
                    else if (*(rangeImage + 199) > 0.6 && !obstacle) {    //przy wewnêtrznym rogu -> szukanie œciany na nowo
                        mode = 9;
                        distance = 0;
                    }
                    else if (*(rangeImage + 199) > *(rangeImage + 196) && *(rangeImage + 199) > 0.4 && *(rangeImage + 199) < 0.6 && !obstacle)    //robot skierowany do œciany -> podje¿d¿anie bli¿ej œciany
                        mode = 8;
                    else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.45 && *(rangeImage + 199) < 0.6) {     //wiêkszy obrót do œciany w przypadku uskoku na œcianie
                        mode = 5;
                        targetAngle = -0.3;
                    }
                    else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.4 && *(rangeImage + 199) <= 0.45) {   //lekki obrót do œciany
                        mode = 5;
                        targetAngle = -0.1;
                    }
                    else {
                        mode = 5;     //lekki obrót od œciany
                        targetAngle = 0.1;
                    }
                }
                else if (cleaning) {
                    robot->pen->write(1);
                    pointX = 1;
                    pointY = 1;
                    if (xd) {
                        xd = false;
                        targetAngle = distMax(pointX, pointY);
                        mode = 10;
                    }
                    else if (xd2) {
                        xd2 = false;
                        targetAngle = distMax(pointX, pointY);
                        mode = 10;
                    }
                }
                else
                    mode = 4;
                startAngle = robot->getPosition()[2];
                break;
        case 2: robot->driveRobot(5);  //jazda prosto z pe³n¹ prêdkoœci¹
                break;
        case 3: robot->driveRobot(2);  //jazda prosto z mniejsz¹ prêdkoœci¹
                break;
        case 4: if (robot->stopRobot())  //zatrzymywanie robota
                    mode = 1;
                break;
        case 5: if (robot->turnRobot(startAngle, targetAngle))      // obrót robota o podany k¹t
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
        case 8: if (*(rangeImage + 199) < 0.6 && *(rangeImage + 199) > 0.4)       // powolna jazda do pewnej wartoœci lidara
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
        case 10: if (robot->turnRobotToAngle(startAngle, targetAngle)) {  // pi / 2, 0
                    if (robot->getPosition()[2] > 6 || robot->getPosition()[2] < 0.1 || (robot->getPosition()[2] > pi - 0.1 && robot->getPosition()[2] < pi + 0.1)) {
                        mode = 13;
                        startCoord = robot->getPosition()[0];
                    }
                    else {
                        mode = 11;
                        startCoord = robot->getPosition()[1];
                    }
                 }
                break;
        case 11: if (abs(pointY - robot->getPosition()[1]) > 0.13)     //dla 2 = 0.13, dla 5 = 0.5
                    robot->driveRobot(2);
               else if ((startCoord < pointY && robot->getPosition()[1] < pointY) || (startCoord > pointY && robot->getPosition()[1] > pointY))
                    robot->driveRobot(0.3);
               else
                    mode = 4;
            break;
        case 12: if (robot->turnRobotToAngle(startAngle, targetAngle))
            mode = 13;
            break;
        case 13: if (abs(pointX - robot->getPosition()[0]) > 0.13)     //dla 2 = 0.13, dla 5 = 0.5
            robot->driveRobot(2);
               else if ((startCoord < pointX && robot->getPosition()[0] < pointX) || (startCoord > pointX && robot->getPosition()[0] > pointX))
            robot->driveRobot(0.3);
               else
            mode = 4;
            break;
        }
    }