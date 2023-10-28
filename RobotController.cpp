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

    double RobotController::distMax(double x, double y) {
        bool isY = true;
        double angle = robot->getPosition()[2];
        if (abs(x - robot->getPosition()[0]) > abs(y - robot->getPosition()[1]))
            isY = false;
        if (isY && abs(y - robot->getPosition()[1]) > 0.006) {
            if (y < robot->getPosition()[1])
                angle = 3 * pi / 2;
            else
                angle = pi / 2;
        }
        else if (!isY && abs(x - robot->getPosition()[0]) > 0.006) {
            if (x < robot->getPosition()[0])
                angle = pi;
            else
                angle = 0;
        }

        return angle;
    }

    void RobotController::convertCoords(double &x, double &y) {
        x = 27.5 + 35 * x;    // 27.5 = 10 + 17.5 czyli promieñ kratki
        y = 27.5 + 35 * y;
        x = (x - (double)map->getMap()[0].size() / 2) / 100;
        y = (y - (double)map->getMap().size() / 2) / -100;
    }

    int RobotController::chooseWay(bool* equalValues, int currentGridX, int currentGridY) {
         if (equalValues[0]) {
             if (map->getObsTransformGrid()[currentGridY][currentGridX - 1] != 30)
                 equalValues[0] = false;
         }
         if (equalValues[1]) {
             if (map->getObsTransformGrid()[currentGridY - 1][currentGridX] != 30)
                 equalValues[1] = false;
         }
         if (equalValues[2]) {
             if (map->getObsTransformGrid()[currentGridY][currentGridX + 1] != 30)
                 equalValues[2] = false;
         }
         if (equalValues[3]) {
             if (map->getObsTransformGrid()[currentGridY + 1][currentGridX] != 30)
                 equalValues[3] = false;
         }

         for (int i = 0; i < 4; i++) {
              if (equalValues[i])       // jeszcze preferowanie jazdy prosto!!!!!!!!!!!
                  return i;
         }
        //std::cout << "eqq " << equalValues[0] << " " << equalValues[1] << " " << equalValues[2] << " " << equalValues[3] << std::endl << std::endl;
    }

    bool RobotController::chooseNext(int &currentGridX, int &currentGridY, double &currentX, double &currentY, std::vector <std::vector<int>> &localGrid) {
        int direction = -1;
        int maxGridValue = -1;
        bool equalValues[4] = { false, false, false, false };   // które wartoœci pól s¹ tak samo du¿e
        bool isEqualValue = false;
        if (currentGridX > 0 && localGrid[currentGridY][currentGridX - 1] >= 0) {       // zachód
            direction = 0;
            maxGridValue = localGrid[currentGridY][currentGridX - 1];
            equalValues[0] = true;
        }
        if (currentGridY > 0 && localGrid[currentGridY - 1][currentGridX] >= 0) {  // pó³noc
            if (localGrid[currentGridY - 1][currentGridX] > maxGridValue) {
                direction = 1;
                maxGridValue = localGrid[currentGridY - 1][currentGridX];
                isEqualValue = false;
                equalValues[0] = false;
                equalValues[1] = true;
            }
            else if (localGrid[currentGridY - 1][currentGridX] == maxGridValue) {
                isEqualValue = true;
                equalValues[1] = true;
            }
        }
        if (currentGridX < localGrid[0].size() - 1 && localGrid[currentGridY][currentGridX + 1] >= 0) {    // wschód
            if (localGrid[currentGridY][currentGridX + 1] > maxGridValue) {
                direction = 2;
                maxGridValue = localGrid[currentGridY][currentGridX + 1];
                isEqualValue = false;
                equalValues[0] = false;
                equalValues[1] = false;
                equalValues[2] = true;
            }
            else if (localGrid[currentGridY][currentGridX + 1] == maxGridValue) {
                isEqualValue = true;
                equalValues[2] = true;
            }
        }
        if (currentGridY < localGrid.size() - 1 && localGrid[currentGridY + 1][currentGridX] >= 0) {   // po³udnie
            if (localGrid[currentGridY + 1][currentGridX] > maxGridValue) {
                direction = 3;
                isEqualValue = false;
            }
            else if (localGrid[currentGridY + 1][currentGridX] == maxGridValue) {
                isEqualValue = true;
                equalValues[3] = true;
            }
        }

        if (isEqualValue)
            direction = chooseWay(equalValues, currentGridX, currentGridY);

        switch (direction) {
        case 0: currentGridX -= 1;              
                break;
        case 1: currentGridY -= 1;
                break;
        case 2: currentGridX += 1;
                break;
        case 3: currentGridY += 1;
                break;
        default: return false;
        }
        currentX = currentGridX;
        currentY = currentGridY;
        convertCoords(currentX, currentY);
        localGrid[currentGridY][currentGridX] = -1;
        return true;
    }

    void RobotController::planPath() {
        std::vector <std::vector<int>> localGrid = map->getGrid();
        double currentX = -1;
        double currentY = -1;
        for (int g = 0; g < localGrid.size(); g++) {
            for (int h = 0; h < localGrid[g].size(); h++) {
                if (localGrid[g][h] == 0) {
                    currentX = h;
                    currentY = g;
                    break;
                }
            }
            if (currentX != -1)
                break;
        }
        int currentGridX = currentX;
        int currentGridY = currentY;
        convertCoords(currentX, currentY);
        path.push_back({currentX, currentY});
        localGrid[currentGridY][currentGridX] = -1;

        while (chooseNext(currentGridX, currentGridY, currentX, currentY, localGrid)) {           
            path.push_back({ currentX, currentY });
        }

        std::cout << "sciezka " << std::endl;
        for (int i = 0; i < path.size(); i++) {   
            std::cout << path[i][0] << " " << path[i][1] << std::endl;
        }
    }

    void RobotController::chooseMode(const float* rangeImage) {
        bool obstacleInFront = false;
        bool obstacle = false;
        
        //double goalX = 0;
        //double goalY = 0;

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
                    bool toPoint = true;
                    if (pathIterator < path.size()) {
                        pointX = path[pathIterator][0];
                        pointY = path[pathIterator][1];
                        if (abs(pointY - robot->getPosition()[1]) > 0.006 || abs(pointX - robot->getPosition()[0]) > 0.006) {
                            targetAngle = distMax(pointX, pointY);
                            mode = 10;  // dziwne obroty o 360 stopni na dole i obrót o 270 zamiast 90 stopni, i usun¹æ nadmiarowe punkty
                        }
                        else
                            pathIterator++;
                    }
                    else
                        mode = 4;
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
        case 13: if (abs(pointX - robot->getPosition()[0]) > 0.13)     //dla 2 = 0.13, dla 5 = 0.5
            robot->driveRobot(2);
               else if ((startCoord < pointX && robot->getPosition()[0] < pointX) || (startCoord > pointX && robot->getPosition()[0] > pointX))
            robot->driveRobot(0.3);
               else
            mode = 4;
            break;
        }
    }