#include "RobotController.hpp"

    RobotController::RobotController(CleaningRobot* cr, Map* m) {
        robot = cr;
        map = m;
    }
    void RobotController::setMode(int m) {
        mode = m;
    }

    bool RobotController::xdd() {
        return xd;
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

    bool RobotController::isGridFinding() {
        return gridFinding;
    }

    void RobotController::setObstacleAvoidance(bool obs) {
        obstacleAvoidance = obs;
        gridFinding = true;
        if (isRoomClean(map->getGrid()))
            exit(0);
        pathIterator = 0;
        path.clear();
    }

    void RobotController::checkObs() {
        if (map->isObstacling() && (abs(map->getObsClosurePosition()[0] - robot->getPosition()[0]) > 0.2 || abs(map->getObsClosurePosition()[1] - robot->getPosition()[1] > 0.2))) {
            map->openObs();
            std::cout << "otwarto\n";
        }
        if (map->isObsOpened() && (abs(map->getObsClosurePosition()[0] - robot->getPosition()[0]) < 0.1) && (abs(map->getObsClosurePosition()[1] - robot->getPosition()[1]) < 0.1)) {
            map->closeObs();
            map->finishObstacling();
            mode = 4;
            std::cout << "zamknieto\n";
        }
    }

    void RobotController::checkMap() {       //!!!!!!!!!!! to te¿ potem mo¿na zmieniæ
        if (!map->isFirstTurn() && (abs(map->getMapClosurePosition()[0] - robot->getPosition()[0]) > 1 || abs(map->getMapClosurePosition()[1] - robot->getPosition()[1] > 1)))
            map->openMap();            //mo¿liwoœæ zamkniêcia pêtli mapy

        if (map->isMapping() && map->isMapOpened() && (abs(map->getMapClosurePosition()[0] - robot->getPosition()[0]) < 0.2) && (abs(map->getMapClosurePosition()[1] - robot->getPosition()[1]) < 0.2)) {
            map->closeMap();
            map->finishMapping();
            mode = 4;
            //robot->clearDisplay();
            //map->printMap();

        }
    }

    void RobotController::checkObstacles2(const float* rangeImage) {    // próbne
        bool obstacles = false;     //czy jakieœ przeszkody s¹ w zasiêgu lidara
       
        if (mode == 2 || mode == 3 || mode == 7 || mode == 8 || mode == 9) {
            if (*(rangeImage + 199) > 0.22 && obstacleAvoidance && mode != 7 && mode != 8 && mode != 9)
                mode = 4;
            else {
                for (int i = 0; i < 200; i++) {
                    if (*(rangeImage + i) < 1) {
                        obstacles = true;
                        if (mode != 7 && mode != 8 && mode != 9)
                            mode = 3;
                        if (map->isObstacling()) {
                            double* xy = robot->calculatePoint(*(rangeImage + i), i);
                            map->insertPoint(xy[0], xy[1]);
                            delete xy;
                        }
                    }
                    if (*(rangeImage + i) < 0.18) {
                        mode = 4;
                        break;
                    }
                }
            }
        }

        if (mode == 3 && obstacles == false)
            mode = 2;
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
                            delete xy;
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
                angle = 2 * pi;
        }

        return angle;
    }

    void RobotController::convertCoords(double &x, double &y) {     // zmiana koordynatów siatki na rzeczywiste
        x = 22.5 + map->getDispX() + 35 * x;    // 22.5 = 5 + 17.5 czyli promieñ kratki
        y = 22.5 + map->getDispY() + 35 * y;//y = (y2 - 22.5 - dispY) / 35
        x = (x - map->getArenaX() / 2) / 100;
        y = (y - map->getArenaY() / 2) / -100;
        //int poseY = -round(positionY * 100) + arenaY / 2;         //positionY = (poseY - arenaY/2) / -100
    }

    bool RobotController::isRoomClean(std::vector <std::vector<int>> grid) {
        for (int g = 0; g < grid.size(); g++) {
            for (int h = 0; h < grid[g].size(); h++) {
                if (grid[g][h] > 0)
                    return false;
            }
        }
        return true;
    }

    std::vector <std::vector<int>> RobotController::createLocalWavePropagation(std::vector <std::vector<int>> grid) {       
        for (int g = 0; g < grid.size(); g++) {
            for (int h = 0; h < grid[g].size(); h++) {
                if (grid[g][h] > 0)
                    grid[g][h] = -1;                
            }
        }
        map->wavePropagation(grid, robot->getPosition()[0], robot->getPosition()[1]);    //zobacz jak z przeszkodami nowymi na œrodku to bd dzia³aæ
        return grid;
    }

    std::vector <std::vector<int>> RobotController::findUnvisitedGrids(std::vector <std::vector<int>> grid) {
        std::vector <std::vector<int>> unvisitedGrids;
        for (int g = 0; g < grid.size(); g++) {
            for (int h = 0; h < grid[g].size(); h++) {
                if (grid[g][h] > 0)
                    unvisitedGrids.push_back({ h, g });   // koordynaty nieodwiedzonych komórek
            }       //UWAGA:: co je¿eli lista pusta?????????/
        }
        return unvisitedGrids;
    }
    
    int* RobotController::getGoalPoint(std::vector <std::vector<int>> localGrid) {
        std::vector <std::vector<int>> unvisitedGrids = findUnvisitedGrids(map->getGrid());
        for (int g = 0; g < unvisitedGrids.size(); g++) {
            unvisitedGrids[g].push_back(localGrid[unvisitedGrids[g][1]][unvisitedGrids[g][0]]);     // zapisanie wartoœci propagacji nieodwiedzonych komórek
        }
        int minValue = 1000;
        int minGrid = 1000;
        for (int g = 0; g < unvisitedGrids.size(); g++) {
            if (unvisitedGrids[g][2] < minValue) {
                minValue = unvisitedGrids[g][2];
                minGrid = g;
            }
        }   // dodaæ przypadek ¿e 2 lub wiêcej s¹ takie same
        int coords[2] = { unvisitedGrids[minGrid][0], unvisitedGrids[minGrid][1] }; //x, y
        return coords;
    }

    void RobotController::planPathToPoint() {
        std::vector <std::vector<int>> localGrid = createLocalWavePropagation(map->getGrid());
        int* goalPoint = getGoalPoint(localGrid);
        int* startPoint = map->getCurrentCell(robot->getPosition()[0], robot->getPosition()[1]);
        std::cout << "goalPoint =   x = " << goalPoint[0] << ",    y = " << goalPoint[1] << std::endl;
        std::cout << "startPoint =   x = " << startPoint[0] << ",    y = " << startPoint[1] << std::endl;

        std::cout << "gridY = " << localGrid.size() << ", gridX = " << localGrid[0].size() << std::endl;
        for (int g = 0; g < localGrid.size(); g++) {
            for (int h = 0; h < localGrid[g].size(); h++) {
                //if (grid[g][h] == -2)
                    //std::cout << "1 ";
                //else
                std::cout << localGrid[g][h] << " ";
            }
            std::cout << std::endl;
        }

        int currentGridX = goalPoint[0];
        int currentGridY = goalPoint[1];
        double currentX = currentGridX;     // wêdrówka od punktu docelowego do startu
        double currentY = currentGridY;
        convertCoords(currentX, currentY);
        path.push_back({ currentX, currentY });

        while (!(currentGridX == startPoint[0] && currentGridY == startPoint[1])) {
            findWayToStart(currentGridX, currentGridY, currentX, currentY, localGrid);
            path.insert(path.begin(), { currentX, currentY });
        }
        delete startPoint;

        optimizePath();

        std::cout << "sciezka " << std::endl;
        for (int i = 0; i < path.size(); i++) {
            std::cout << path[i][0] << " " << path[i][1] << std::endl;
        }
        gridFinding = false;
    }

    void RobotController::findWayToStart(int& currentGridX, int& currentGridY, double& currentX, double& currentY, std::vector <std::vector<int>>& localGrid) {
        int propagationValue = localGrid[currentGridY][currentGridX];
        if (currentGridX > 0 && localGrid[currentGridY][currentGridX - 1] == propagationValue - 1)     // zachód
            currentGridX -= 1;       
        else if (currentGridY > 0 && localGrid[currentGridY - 1][currentGridX] == propagationValue - 1)  // pó³noc
            currentGridY -= 1;
        else if (currentGridX < localGrid[0].size() - 1 && localGrid[currentGridY][currentGridX + 1] == propagationValue - 1)    // wschód
            currentGridX += 1;
        else if (currentGridY < localGrid.size() - 1 && localGrid[currentGridY + 1][currentGridX] == propagationValue - 1)   // po³udnie
            currentGridY += 1;

        currentX = currentGridX;
        currentY = currentGridY;
        convertCoords(currentX, currentY);
    }

    void RobotController::occupyVisitedCells() {
        int* currentPoint = map->getCurrentCell(robot->getPosition()[0], robot->getPosition()[1]);
        int* startPoint;
        if (pathIterator == 0)
            startPoint = map->getCurrentCell(path[pathIterator][0], path[pathIterator][1]);
        else
            startPoint = map->getCurrentCell(path[pathIterator - 1][0], path[pathIterator - 1][1]);

        map->setGridCell(currentPoint[0], currentPoint[1], -1);
        if (startPoint[0] == currentPoint[0]) {     // je¿eli œcie¿ka idzie pionowo wzd³u¿ y
            while (currentPoint[1] != startPoint[1]) {
                if (startPoint[1] > currentPoint[1])
                    currentPoint[1] += 1;
                else if (startPoint[1] < currentPoint[1])
                    currentPoint[1] -= 1;
                else
                    break;
                map->setGridCell(currentPoint[0], currentPoint[1], -1);
            }
        }
        else {                                                  // je¿eli œcie¿ka idzie poziomo wzd³u¿ x
            while (currentPoint[0] != startPoint[0]) {
                if (startPoint[0] > currentPoint[0])
                    currentPoint[0] += 1;
                else if (startPoint[0] < currentPoint[0])
                    currentPoint[0] -= 1;
                else
                    break;
                map->setGridCell(currentPoint[0], currentPoint[1], -1);
            }
        }
        delete currentPoint, startPoint;
    }

    int RobotController::chooseWay(bool* equalValues, int currentGridX, int currentGridY, double currentX, double currentY) {
        if (path.size() > 1 && path[path.size() - 2][1] == currentY && (equalValues[0] || equalValues[2])) {  // je¿eli y jest sta³e i ruch jest horyzontalny
            if (equalValues[0])
                return 0;
            else
                return 2;
        }
        else if (path.size() > 1 && path[path.size() - 2][0] == currentX && (equalValues[1] || equalValues[3])) {  // je¿eli x jest sta³e i ruch jest wertykalny
            if (equalValues[1])
                return 1;
            else
                return 3;
        }

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

         int counter = 0;
         for (int i = 0; i < 4; i++) {
             if (equalValues[i])
                 counter++;
         }

         if (counter == 1) {
             for (int i = 0; i < 4; i++) {
                 if (equalValues[i])
                     return i;
             }
         }
         else {
                 for (int i = 0; i < 4; i++) {
                     if (equalValues[i])
                         return i;
                 }
             
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
            direction = chooseWay(equalValues, currentGridX, currentGridY, currentX, currentY);

        switch (direction) {
        case 0: currentGridX -= 1;              
                break;
        case 1: currentGridY -= 1;
                break;
        case 2: currentGridX += 1;
                break;
        case 3: currentGridY += 1;
                break;
        default: return false;////
        }
        currentX = currentGridX;
        currentY = currentGridY;
        convertCoords(currentX, currentY);
        localGrid[currentGridY][currentGridX] = -1;
        return true;
    }

    void RobotController::optimizePath() {
        for (int i = 0; i < path.size() - 2; i++) {
            if ((path[i][0] == path[i + 1][0] && path[i + 1][0] == path[i + 2][0]) || (path[i][1] == path[i + 1][1] && path[i + 1][1] == path[i + 2][1])) {
                path.erase(path.begin() + i + 1);
                i--;
            }
        }
    }

    void RobotController::planPath() {
        std::vector <std::vector<int>> localGrid = map->getGrid();
        double currentX = map->getCurrentCell(robot->getPosition()[0], robot->getPosition()[1])[0];
        double currentY = map->getCurrentCell(robot->getPosition()[0], robot->getPosition()[1])[1];
        /*for (int g = 0; g < localGrid.size(); g++) {
            for (int h = 0; h < localGrid[g].size(); h++) {
                if (localGrid[g][h] == 0) {
                    currentX = h;
                    currentY = g;
                    break;
                }
            }
            if (currentX != -1)
                break;
        }*/
        int currentGridX = currentX;
        int currentGridY = currentY;
        convertCoords(currentX, currentY);
        path.push_back({currentX, currentY});
        localGrid[currentGridY][currentGridX] = -1;

        while (chooseNext(currentGridX, currentGridY, currentX, currentY, localGrid)) {           
            path.push_back({ currentX, currentY });
        }
        optimizePath();

        std::cout << "sciezka " << std::endl;
        for (int i = 0; i < path.size(); i++) {   
            std::cout << path[i][0] << " " << path[i][1] << std::endl;
        }

        gridFinding = false;
        //map->setGrid(localGrid);    // uwaga!!! to z³e podejœcie przy uwzglêdnianiu przeszkód na œrodku!!!!!
    }

    bool RobotController::isObstacleOnLidar(const float* rangeImage) {
        for (int i = 0; i < 200; i++) {
            if (*(rangeImage + i) < 1)
                return true;
        }
        return false;
    }

    bool RobotController::checkObstacleTransform() {
        int* currentPoint = map->getCurrentCell(robot->getPosition()[0], robot->getPosition()[1]);
        if (map->getObsTransformGrid()[currentPoint[1]][currentPoint[0]] == 30 || map->getObsTransformGrid()[currentPoint[1]][currentPoint[0]] == -2) {
            delete currentPoint;
            return true;
        }
        delete currentPoint;
        return false;
    }

    void RobotController::Lidar(const float* rangeImage) {
        for (int i = 0; i < 200; i++) {
            if (*(rangeImage + i) < 0.18) {               
                mode = 4;
                obstacleAvoidance = true;
                occupyVisitedCells();
                
            }
        }
    }

    bool RobotController::isObstacleAvoidance() {
        return obstacleAvoidance;
    }

    void RobotController::chooseMode(const float* rangeImage) {
        bool obstacleInFront = false;
        bool obstacle = false;
        

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
                    else if (*(rangeImage + 199) > *(rangeImage + 196) && *(rangeImage + 199) > 0.4 && *(rangeImage + 199) < 0.6 && !obstacle) {    //robot skierowany do œciany -> podje¿d¿anie bli¿ej œciany
                        mode = 8;
                        cond1 = 0.6;
                        cond2 = 0.4;
                    }
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
                else if (cleaning && obstacleAvoidance) {
                    if (xd) {
                        mode = 12;
                        map->setObsClosurePosition(robot->getPosition()[0], robot->getPosition()[1]);
                        map->beginObstacling();
                        std::cout << "zamkniecie: x = " << robot->getPosition()[0] << "   y = " << robot->getPosition()[1] << std::endl;
                        xd = false;
                    }
                    else {
                        for (int i = 0; i < 200; i++) {
                            if (*(rangeImage + i) < 0.18) {
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
                        else if (*(rangeImage + 199) > 0.27 && !obstacle) {    //przy wewnêtrznym rogu -> szukanie œciany na nowo
                            mode = 9;
                            distance = 0;
                        }
                        else if (*(rangeImage + 199) > *(rangeImage + 196) && *(rangeImage + 199) > 0.22 && *(rangeImage + 199) < 0.27 && !obstacle) {    //robot skierowany do œciany -> podje¿d¿anie bli¿ej œciany
                            mode = 8;
                            cond1 = 0.27;
                            cond2 = 0.22;
                        }
                        else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.24 && *(rangeImage + 199) < 0.27) {     //wiêkszy obrót do œciany w przypadku uskoku na œcianie
                            mode = 5;
                            targetAngle = -0.3;
                        }
                        else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.22 && *(rangeImage + 199) <= 0.24) {   //lekki obrót do œciany
                            mode = 5;
                            targetAngle = -0.1;
                        }
                        else {
                            mode = 5;     //lekki obrót od œciany
                            targetAngle = 0.1;
                        }
                    }
                }
                else if (cleaning) {
                    robot->pen->write(1);
                    if (pathIterator < path.size()) {
                        pointX = path[pathIterator][0];
                        pointY = path[pathIterator][1];
                        if (abs(pointY - robot->getPosition()[1]) > 0.006 || abs(pointX - robot->getPosition()[0]) > 0.006) {
                            targetAngle = distMax(pointX, pointY);
                            mode = 10;  // dziwne obroty o 360 stopni na dole i obrót o 270 zamiast 90 stopni, i usun¹æ nadmiarowe punkty
                        }
                        else {
                            //map->setGridCell(map->getCurrentCell(robot->getPosition()[0], robot->getPosition()[1])[0], map->getCurrentCell(robot->getPosition()[0], robot->getPosition()[1])[1], -1);
                            occupyVisitedCells();
                            pathIterator++;     // ustawia tylko punkty koñcowe a powinno te¿ wczeœniejsze

                            std::cout << "gridY = " << map->getGrid().size() << ", gridX = " << map->getGrid()[0].size() << std::endl;
                            for (int g = 0; g < map->getGrid().size(); g++) {
                                for (int h = 0; h < map->getGrid()[g].size(); h++) {
                                    //if (grid[g][h] == -2)
                                        //std::cout << "1 ";
                                    //else
                                    std::cout << map->getGrid()[g][h] << " ";
                                }
                                std::cout << std::endl;
                            }
                        }
                    }
                    else {
                        if (isRoomClean(map->getGrid()))
                            exit(0);
                        pathIterator = 0;
                        path.clear();
                        gridFinding = true;
                        mode = 4;                      
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
        case 6: if (robot->turnRobot(startAngle, -pi / 2)) {
            mode = 7;
            if (obstacleAvoidance)
                cond1 = 0.27;
            else
                cond1 = 0.6;
        }
                break;
        case 7: if (*(rangeImage + 199) > cond1)//0.6
                    robot->driveRobot(0.5);
                else
                    mode = 4;
                break;
        case 8: if (*(rangeImage + 199) < cond1 && *(rangeImage + 199) > cond2)//0.6 0.4       // powolna jazda do pewnej wartoœci lidara
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
        case 11: if (abs(pointY - robot->getPosition()[1]) > 0.55 && !isObstacleOnLidar(rangeImage))
                    robot->driveRobot(5);
                else if (abs(pointY - robot->getPosition()[1]) > 0.13)     //dla 2 = 0.13, dla 5 = 0.5
                    robot->driveRobot(2);
               else if ((startCoord < pointY && robot->getPosition()[1] < pointY) || (startCoord > pointY && robot->getPosition()[1] > pointY))
                    robot->driveRobot(0.3);
               else
                    mode = 4;
            break;
        case 13: if (abs(pointX - robot->getPosition()[0]) > 0.55 && !isObstacleOnLidar(rangeImage))
                    robot->driveRobot(5);          
                else if (abs(pointX - robot->getPosition()[0]) > 0.13)     //dla 2 = 0.13, dla 5 = 0.5
                    robot->driveRobot(2);
               else if ((startCoord < pointX && robot->getPosition()[0] < pointX) || (startCoord > pointX && robot->getPosition()[0] > pointX))
            robot->driveRobot(0.3);
               else
            mode = 4;
            break;
        case 12: if (*(rangeImage + 199) > 0.2)
            robot->turnRobot(startAngle, 2 * pi);
               else
            mode = 4;
            break;
        }
    }