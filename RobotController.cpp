#include "RobotController.hpp"

    RobotController::RobotController(CleaningRobot* cr, Map* m) {
        robot = cr;
        map = m;
    }

    void RobotController::setMode(int m) {
        mode = m;
    }

    void RobotController::setPathConditions(bool c) {
        extraPathConditions = c;
    }

    bool RobotController::isFirstObstacleRotation() {
        return firstObstacleRotation;
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

    int RobotController::getDistanceTraveled() {
        return distanceTraveled;
    }

    int RobotController::getTotalRotates() {
        return totalRotates;
    }

    bool RobotController::isObstacleAvoidance() {
        return obstacleAvoidance;
    }

    void RobotController::setObstacleAvoidance(bool obs) {
        obstacleAvoidance = obs;
    }

    void RobotController::finishObstacleAvoidance() {
        gridFinding = true;
        pathIterator = 0;
        path.clear();
        firstObstacleRotation = true;
    }

    void RobotController::checkObstacleCompletion() {
        if (map->isObstacling() && (abs(map->getObsClosurePosition()[0] - robot->getPosition()[0]) > 0.2 || abs(map->getObsClosurePosition()[1] - robot->getPosition()[1] > 0.2))) {
            map->openObs();
        }
        if (map->isObsOpened() && (abs(map->getObsClosurePosition()[0] - robot->getPosition()[0]) < 0.1) && (abs(map->getObsClosurePosition()[1] - robot->getPosition()[1]) < 0.1)) {
            map->closeObs();
            map->finishObstacling();
            mode = 4;
        }
    }

    void RobotController::checkMap() {
        if (!map->isFirstRotation() && (abs(map->getMapClosurePosition()[0] - robot->getPosition()[0]) > 1 || abs(map->getMapClosurePosition()[1] - robot->getPosition()[1] > 1)))
            map->openMap();            //mo¿liwoœæ zamkniêcia pêtli mapy

        if (map->isMapping() && map->isMapOpened() && (abs(map->getMapClosurePosition()[0] - robot->getPosition()[0]) < 0.2) && (abs(map->getMapClosurePosition()[1] - robot->getPosition()[1]) < 0.2)) {
            map->closeMap();
            map->finishMapping();
            mode = 4;
        }
    }

    void RobotController::checkObstacle(const float* rangeImage) {    // sprawdzanie przeszkody podczas omijania 
        if (mode == 2 || mode == 3 || mode == 7 || mode == 8 || mode == 9) {
            if (*(rangeImage + 199) > 0.24 && obstacleAvoidance && mode != 7 && mode != 8 && mode != 9) //0.4 -> 0.22, 0.2->0.18, 0.6->0.27, 0.45->0.24
                mode = 4;
            else {
                for (int i = 0; i < 200; i++) {
                    if (*(rangeImage + i) < robot->getLidarRange()) {
                        if (map->isObstacling()) {
                            double* xy = robot->calculatePoint(*(rangeImage + i), i);
                            map->insertPoint(xy[0], xy[1]);
                            delete[] xy;
                        }
                    }
                    if (*(rangeImage + i) < 0.2) {
                        mode = 4;
                        break;
                    }
                }
            }
        }
    }

    void RobotController::checkObstacles(const float* rangeImage) {    // sprawdzenie czy s¹ przeszkody podczas obje¿d¿ania pokoju  
        if (mode == 2 || mode == 3 || mode == 7 || mode == 8 || mode == 9) {    //sprawdzanie przeszkód tylko gdy robot nie skrêca
            if (*(rangeImage + 199) > 0.4 && map->isMapping() && mode != 7 && mode != 8 && mode != 9)
                mode = 4;
            else {
                for (int i = 0; i < 200; i++) {
                    if (*(rangeImage + i) < robot->getLidarRange()) {
                        if (map->isMapping()) {
                            double* xy = robot->calculatePoint(*(rangeImage + i), i);
                            map->insertPoint(xy[0], xy[1]);
                            delete[] xy;
                        }
                    }
                    if (*(rangeImage + i) < 0.2) {      //je¿eli przeszkoda jest zbyt blisko = stop
                        mode = 4;
                        if (!map->isWallFound())
                            map->beginMapping();
                        map->setWallFound();
                        break;
                    }
                }
            }
        }
    }

    double RobotController::getTargetAngle(double x, double y) {
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
        y = 22.5 + map->getDispY() + 35 * y;
        x = (x - map->getArenaX() / 2) / 100;
        y = (y - map->getArenaY() / 2) / -100;
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
            for (int h = 0; h < grid[g].size(); h++) {      // zabranie wag kratkom nieodwiedzonym
                if (grid[g][h] > 0)
                    grid[g][h] = -1;                
            }
        }
        map->wavePropagation(grid, robot->getPosition()[0], robot->getPosition()[1]);
        return grid;
    }

    std::vector <std::vector<int>> RobotController::findUnvisitedGrids(std::vector <std::vector<int>> grid) {
        std::vector <std::vector<int>> unvisitedGrids;
        for (int g = 0; g < grid.size(); g++) {
            for (int h = 0; h < grid[g].size(); h++) {
                if (grid[g][h] > 0)
                    unvisitedGrids.push_back({ h, g });   // koordynaty nieodwiedzonych komórek
            }
        }
        return unvisitedGrids;
    }
    
    int* RobotController::getGoalPoint(std::vector <std::vector<int>> localGrid) {       // punkt docelowy w replanningu œcie¿ki
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
        }
        int coords[2] = { unvisitedGrids[minGrid][0], unvisitedGrids[minGrid][1] };
        return coords;
    }

    void RobotController::planPathToPoint() {
        std::vector <std::vector<int>> localGrid = createLocalWavePropagation(map->getGrid());
        int* goalPoint = getGoalPoint(localGrid);
        int* startPoint = map->getCurrentCell(robot->getPosition()[0], robot->getPosition()[1]);

        std::cout << "gridY = " << localGrid.size() << ", gridX = " << localGrid[0].size() << std::endl;
        for (int g = 0; g < localGrid.size(); g++) {
            for (int h = 0; h < localGrid[g].size(); h++) {
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

        optimizePath();

        if (map->getGrid()[startPoint[1]][startPoint[0]] == -2)     // je¿eli robot jest na zajêtej komórce po omijaniu przeszkody
            path.erase(path.begin());

        delete[] startPoint;
        std::cout << "sciezka: " << std::endl;
        for (int i = 0; i < path.size(); i++) {
            std::cout << path[i][0] << " " << path[i][1] << std::endl;
        }
        gridFinding = false;
        totalRotates += path.size() - 1;
        if ((path[0][0] > path[1][0] && robot->getPosition()[2] > 3 && robot->getPosition()[2] < 3.3)     // zachód
            || (path[0][1] < path[1][1] && robot->getPosition()[2] > 1.4 && robot->getPosition()[2] < 1.8)  // pó³noc
            || (path[0][0] < path[1][0] && (robot->getPosition()[2] > 6 || robot->getPosition()[2] < 0.3))  // wschód
            || (path[0][1] > path[1][1] && robot->getPosition()[2] > 4.5 && robot->getPosition()[2] < 4.9)) // po³udnie
            totalRotates--;
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

    void RobotController::occupyVisitedCells() {    // po osi¹gniêciu ka¿dego punktu œcie¿ki zaznaczanie odwiedzonych komórek
        int* currentPoint = map->getCurrentCell(robot->getPosition()[0], robot->getPosition()[1]);
        int* startPoint;
        if (pathIterator == 0)
            startPoint = map->getCurrentCell(path[pathIterator][0], path[pathIterator][1]);
        else
            startPoint = map->getCurrentCell(path[pathIterator - 1][0], path[pathIterator - 1][1]);

        map->setGridCell(currentPoint[0], currentPoint[1], -1);
        if (startPoint[0] == currentPoint[0]) {     // je¿eli œcie¿ka idzie pionowo wzd³u¿ y
            while (currentPoint[1] != startPoint[1]) {
                if (startPoint[1] > currentPoint[1]) {
                    currentPoint[1] += 1;
                    distanceTraveled++;
                }
                else if (startPoint[1] < currentPoint[1]) {
                    currentPoint[1] -= 1;
                    distanceTraveled++;
                }
                else
                    break;
                map->setGridCell(currentPoint[0], currentPoint[1], -1);
            }
        }
        else {                                                  // je¿eli œcie¿ka idzie poziomo wzd³u¿ x
            while (currentPoint[0] != startPoint[0]) {
                if (startPoint[0] > currentPoint[0]) {
                    currentPoint[0] += 1;
                    distanceTraveled++;
                }
                else if (startPoint[0] < currentPoint[0]) {
                    currentPoint[0] -= 1;
                    distanceTraveled++;
                }
                else
                    break;
                map->setGridCell(currentPoint[0], currentPoint[1], -1);
            }
        }
        delete[] currentPoint, startPoint;
    }

    int RobotController::considerPathConditions(bool* equalValues, int currentGridX, int currentGridY, double currentX, double currentY) {
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
    }

    bool RobotController::chooseNextPoint(int &currentGridX, int &currentGridY, double &currentX, double &currentY, std::vector <std::vector<int>> &localGrid) {
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

        if (isEqualValue && extraPathConditions)
            direction = considerPathConditions(equalValues, currentGridX, currentGridY, currentX, currentY);
        else if (isEqualValue && !extraPathConditions) {
            for (int i = 0; i < 4; i++) {
                if (equalValues[i]) {
                    direction = i;
                    break;
                }
            }
        }

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
        int* currentGrid = map->getCurrentCell(robot->getPosition()[0], robot->getPosition()[1]);
        int currentGridX = currentGrid[0];
        int currentGridY = currentGrid[1];
        delete[] currentGrid;
        
        double currentX = currentGridX;
        double currentY = currentGridY;
        convertCoords(currentX, currentY);
        path.push_back({currentX, currentY});
        localGrid[currentGridY][currentGridX] = -1;

        while (chooseNextPoint(currentGridX, currentGridY, currentX, currentY, localGrid)) {           
            path.push_back({ currentX, currentY });
        }
        optimizePath();

        std::cout << "sciezka: " << std::endl;
        for (int i = 0; i < path.size(); i++) {   
            std::cout << path[i][0] << " " << path[i][1] << std::endl;
        }

        gridFinding = false;
        totalRotates += path.size() - 1;
        if ((path[0][0] > path[1][0] && robot->getPosition()[2] > 3 && robot->getPosition()[2] < 3.3)     // zachód
            || (path[0][1] < path[1][1] && robot->getPosition()[2] > 1.4 && robot->getPosition()[2] < 1.8)  // pó³noc
            || (path[0][0] < path[1][0] && (robot->getPosition()[2] > 6 || robot->getPosition()[2] < 0.3))  // wschód
            || (path[0][1] > path[1][1] && robot->getPosition()[2] > 4.5 && robot->getPosition()[2] < 4.9)) // po³udnie
            totalRotates--;
    }

    bool RobotController::isObstacleOnLidar(const float* rangeImage) {
        for (int i = 0; i < 200; i++) {
            if (*(rangeImage + i) < 1)
                return true;
        }
        return false;
    }

    bool RobotController::checkObstacleTransform() {    // czy robot znajduje siê w s¹siedztwie znanej przeszkody
        int* currentPoint = map->getCurrentCell(robot->getPosition()[0], robot->getPosition()[1]);
        if (map->getObsTransformGrid()[currentPoint[1]][currentPoint[0]] == 30 || map->getObsTransformGrid()[currentPoint[1]][currentPoint[0]] == -2) {
            delete[] currentPoint;
            return true;
        }
        delete[] currentPoint;
        return false;
    }

    void RobotController::checkUnexpectedObstacles(const float* rangeImage) {
        for (int i = 0; i < 200; i++) {
            if (*(rangeImage + i) < 0.18) {               
                mode = 4;
                obstacleAvoidance = true;
                occupyVisitedCells();               
            }
        }
    }

    void RobotController::obstaclesWhenRobotIsStopped(bool &obstacleInFront, bool &obstacle, const float* rangeImage) {
        for (int i = 0; i < 200; i++) {
            if (*(rangeImage + i) < 0.2) {
                obstacle = true;
                if (i > 90 && i < 110) {
                    obstacleInFront = true;
                    break;
                }
            }
        }
    }

    void RobotController::obstacleFollowing(const float* rangeImage) {
        bool obstacleInFront = false;
        bool obstacle = false;
        obstaclesWhenRobotIsStopped(obstacleInFront, obstacle, rangeImage);
        
        if (obstacleInFront) {    //przy rogu -> obrót o 90 stopni
            mode = 5;
            targetAngle = pi / 2;
        }
        else if (*(rangeImage + 199) > 0.29 && !obstacle) {    //przy wewnêtrznym rogu -> szukanie przeszkody na nowo
            mode = 9;
            distance = 0;
        }
        else if (*(rangeImage + 199) > *(rangeImage + 196) && *(rangeImage + 199) > 0.24 && *(rangeImage + 199) < 0.29 && !obstacle) {    //robot skierowany do przeszkody -> podje¿d¿anie bli¿ej
            mode = 8;
            cond1 = 0.29;
            cond2 = 0.24;
        }
        else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.26 && *(rangeImage + 199) < 0.29) {     //wiêkszy obrót do przeszkody w przypadku uskoku
            mode = 5;
            targetAngle = -0.3;
        }
        else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.24 && *(rangeImage + 199) <= 0.26) {   //lekki obrót do przeszkody
            mode = 5;
            targetAngle = -0.1;
        }
        else {
            mode = 5;     //lekki obrót od przeszkody
            targetAngle = 0.11;
        }
    }

    void RobotController::wallFollowing(const float* rangeImage) {
        bool obstacleInFront = false;
        bool obstacle = false;
        obstaclesWhenRobotIsStopped(obstacleInFront, obstacle, rangeImage);
        
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
            targetAngle = 0.11;
        }
    }

    void RobotController::chooseMode(const float* rangeImage) {      
        switch (mode) {
            // tryb decyzyjny
        case 1: if (map->isMapping() && map->isFirstRotation()) {    // pocz¹tek mapowania
                    map->finishFirstRotation();
                    map->setMapClosurePosition(robot->getPosition()[0], robot->getPosition()[1]);     // wspó³rzêdne rozpoczynaj¹ce mapowanie
                }
                else if (map->isMapping() && !map->isFirstRotation()) {     // polecenia przy tworzeniu mapy
                    wallFollowing(rangeImage);
                }
                else if (cleaning && obstacleAvoidance) {       // omijanie przeszkody
                    if (firstObstacleRotation) {
                        mode = 12;
                        map->setObsClosurePosition(robot->getPosition()[0], robot->getPosition()[1]);
                        map->beginObstacling();
                        firstObstacleRotation = false;
                    }
                    else
                        obstacleFollowing(rangeImage);                 
                }
                else if (cleaning) {    // pod¹¿anie œcie¿k¹
                    robot->pen->write(1);
                    robot->disablePointCloud();
                    if (pathIterator < path.size()) {
                        pointX = path[pathIterator][0];
                        pointY = path[pathIterator][1];
                        if (abs(pointY - robot->getPosition()[1]) > 0.006 || abs(pointX - robot->getPosition()[0]) > 0.006) {
                            targetAngle = getTargetAngle(pointX, pointY);
                            mode = 10;
                        }
                        else {
                            occupyVisitedCells();
                            pathIterator++;

                            std::cout << "gridY = " << map->getGrid().size() << ", gridX = " << map->getGrid()[0].size() << std::endl;
                            for (int g = 0; g < map->getGrid().size(); g++) {
                                for (int h = 0; h < map->getGrid()[g].size(); h++) {
                                    std::cout << map->getGrid()[g][h] << " ";
                                }
                                std::cout << std::endl;
                            }
                        }
                    }
                    else {
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
        case 2: robot->driveRobot(5);  // jazda prosto z pe³n¹ prêdkoœci¹
                break;
        case 3: robot->driveRobot(2);  // jazda prosto z mniejsz¹ prêdkoœci¹
                break;
        case 4: if (robot->stopRobot())  // zatrzymywanie robota
                    mode = 1;
                break;
        case 5: if (robot->rotateRobot(startAngle, targetAngle))      // obrót robota o podany k¹t
                    mode = 3;
                break;
        case 6: if (robot->rotateRobot(startAngle, -pi / 2)) {      // obrót o 90 stopni w prawo
                    mode = 7;
                    if (obstacleAvoidance)
                        cond1 = 0.29;
                    else
                        cond1 = 0.6;
                }
                break;
        case 7: if (*(rangeImage + 199) > cond1)
                    robot->driveRobot(0.5);
                else
                    mode = 4;
                break;
        case 8: if (*(rangeImage + 199) < cond1 && *(rangeImage + 199) > cond2)      // powolna jazda do pewnej wartoœci lidara
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
        case 10: if (robot->rotateRobotToAngle(startAngle, targetAngle)) {      // obrót robota o konkretny k¹t
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
        case 11: if (abs(pointY - robot->getPosition()[1]) > 0.55 && !isObstacleOnLidar(rangeImage))    // jazda do docelowego Y
                    robot->driveRobot(5);
                 else if (abs(pointY - robot->getPosition()[1]) > 0.13)
                    robot->driveRobot(2);
                 else if ((startCoord < pointY && robot->getPosition()[1] < pointY) || (startCoord > pointY && robot->getPosition()[1] > pointY))
                    robot->driveRobot(0.3);
                 else
                    mode = 4;
                 break;
        case 12: if (*(rangeImage + 199) > 0.29)
                    robot->rotateRobot(startAngle, 2 * pi);
                 else
                    mode = 4;
                 break;
        case 13: if (abs(pointX - robot->getPosition()[0]) > 0.55 && !isObstacleOnLidar(rangeImage))       // jazda do docelowego X
                    robot->driveRobot(5);          
                 else if (abs(pointX - robot->getPosition()[0]) > 0.13)
                    robot->driveRobot(2);
                 else if ((startCoord < pointX && robot->getPosition()[0] < pointX) || (startCoord > pointX && robot->getPosition()[0] > pointX))
                    robot->driveRobot(0.3);
                 else
                    mode = 4;
                 break;     
        }
    }