#include "Map.hpp"
#include <vector>
#include <iostream>

    void Map::beginMapping() {
        mapping = true;
    }
    void Map::finishMapping() {
        mapping = false;
    }
    bool Map::isMapping() {
        return mapping;
    }
    bool Map::isWallFound() {
        return wallFound;
    }
    void Map::setWallFound() {
        wallFound = true;
    }
    double* Map::getMapClosurePosition() {
        return mapClosurePosition;
    }
    void Map::openMap() {
        mapOpened = true;
    }
    void Map::closeMap() {
        mapOpened = false;
    }
    bool Map::isMapOpened() {
        return mapOpened;
    }
    void Map::setMapClosurePosition(double x, double y) {
        mapClosurePosition[0] = x;
        mapClosurePosition[1] = y;
    }
    bool Map::isFirstTurn() {
        return firstTurn;
    }
    void Map::finishFirstTurn() {
        firstTurn = false;
    }
    bool Map::isPoint(double x, double y) {
        for (int i = 0; i < points.size(); i++)
        {
            if (points[i][0] == x && points[i][1] == y)
                return true;
        }
        return false;
    }
    void Map::insertPoint(double x, double y) {
        if (!isPoint(x, y))
            points.push_back({x, y});
    }
    void Map::createMap(double x, double y) {     // tworzenie mapy (siatki zajêtoœci), true = przeszkoda
        double minX = 0;
        double maxX = 0;
        double minY = 0;
        double maxY = 0;
        for (int i = 0; i < points.size(); i++)
        {
            if (points[i][0] < minX)
                minX = points[i][0];
            if (points[i][0] > maxX)
                maxX = points[i][0];
            if (points[i][1] < minY)
                minY = points[i][1];
            if (points[i][1] > maxY)
                maxY = points[i][1];
        }
        minX = round(minX * 100);
        maxX = round(maxX * 100);
        minY = round(minY * 100);
        maxY = round(maxY * 100);
        std::cout << "maxY " << maxY << std::endl;
        
        int xsize = int(maxX - minX);
        int ysize = int(maxY - minY);
        //std::vector<std::vector<bool>> v(arenaY, std::vector<bool>(arenaX, false));
        globalMap = new bool* [arenaY];
        for (int i = 0; i < arenaY; i++) {
            globalMap[i] = new bool[arenaX];
            
            //std::cout << std::endl;
        }
        for (int i = 0; i < arenaY; i++) {
            for (int j = 0; j < arenaX; j++)
                globalMap[i][j] = false;

            //std::cout << std::endl;
        }
        
        
        for (int j = 0; j < points.size(); j++) {
            //std::cout << j << std::endl;
            int wyrY = -round(points[j][1] * 100) + arenaY / 2;
            int wyrX = round(points[j][0] * 100) + arenaX / 2;
            //std::cout << -round(points[j][1] * 100) + arenaY / 2 - 1 << std::endl;
            if (wyrY >= arenaY) {
                globalMap[arenaY - 1][wyrX] = true;
                //globalMap[int(-round(points[j][1] * 100)) + int(arenaY / 2) - 1][round(points[j][0] * 100) - arenaX / 2] = true;
                //std::cout << "lol " << int(-round(points[j][1] * 100)) + int(arenaY / 2) - 1 << std::endl;
                
            }
            else if (wyrY < 0)
                globalMap[0][wyrX] = true;
            else if (wyrX >= arenaX) {
                globalMap[wyrY][arenaX - 1] = true;
                //std::cout << "lol " << arenaX - 1 << std::endl;
            }
            else if (wyrX < 0)
                globalMap[wyrY][0] = true;
            else
                globalMap[wyrY][wyrX] = true;
            //for (int f = 0; f < globalMap[0].size(); f++)
                //std::cout << globalMap[399][f] << " ";
            //std::cout << std::endl;
        }

        optimizeMap();
        createGrid(x, y);
        std::cout << "rozmiar mapy: x = " << map[0].size() << "   , y = " << map.size() << std::endl;
    }

    void Map::setMapCorrectionValue(int value) {
        mapCorrectionValue = value;
    }

    void Map::optimizeMap() {
        std::vector<std::vector<bool>> v(arenaY, std::vector<bool>(arenaX, false));
        map = v;
        for (int i = 0; i < map.size(); i++) {
            for (int j = 0; j < map[i].size(); j++)
                map[i][j] = globalMap[i][j];
        }

        bool displacement = true;

        int counter = 0;
        for (int i = 0; i < map.size(); i++) {
            for (int j = 0; j < map[i].size(); j++) {
                if (!map[i][j])
                    counter++;
                else if (map[i][j] && counter <= mapCorrectionValue) {
                    for (int k = 1; k <= counter; k++)
                        map[i][j - k] = true;
                    counter = 0;
                }
                else
                    counter = 0;
            }
            if (counter == map[i].size()) {
                map.erase(map.begin() + i);
                if (displacement)
                    displacementY++;
                i--;
            }
            else
                displacement = false;
            counter = 0;
        }
        counter = 0;
        displacement = true;
        for (int j = 0; j < map[0].size(); j++) {
            for (int i = 0; i < map.size(); i++) {
                if (!map[i][j])
                    counter++;
                else if (map[i][j] && counter <= mapCorrectionValue) {
                    for (int k = 1; k <= counter; k++) {
                        if (i - k >= 0)
                            map[i - k][j] = true;
                    }
                    counter = 0;
                }
                else
                    counter = 0;
            }
            if (counter == map.size()) {
                for (int g = 0; g < map.size(); g++)
                    map[g].erase(map[g].begin() + j);
                if (displacement)
                    displacementX++;
                j--;
            }
            else
                displacement = false;
            counter = 0;
        }
    }

    void Map::createGrid(double x, double y) {
        int xsize = (map[0].size() - 5) / 35;
        std::cout << xsize;
        std::cout << std::endl << (map[0].size() - 5) / 35;
        //if (double((map[0].size() - 5) / 35) - xsize < 0.15)        // je¿eli ostatnia kratka jest bli¿ej ni¿ 5 jednostek od œciany
            //xsize -= 1;
        int ysize = (map.size() - 5) / 35;
        //if (double((map.size() - 5) / 35) - ysize < 0.15)
            //ysize -= 1;
        gridSizeX = xsize;
        gridSizeY = ysize;
        std::cout << "gridSizeX = " << xsize << "   gridSizeY = " << ysize << std::endl;
        std::cout << "gridSizeX = " << gridSizeX << "   gridSizeY = " << gridSizeY << std::endl;
        std::vector<std::vector<int>> v(ysize, std::vector<int>(xsize, -1));
        grid = v;

        bool gridFull;
        for (int g = 0; g < grid.size(); g++) {
            for (int h = 0; h < grid[g].size(); h++) {
                gridFull = false;
                for (int i = 5 + g * 35; i < 40 + g * 35; i++) {
                    for (int j = 5 + h * 35; j < 40 + h * 35; j++) {
                        if (map[i][j]) {
                            grid[g][h] = -2;
                            gridFull = true;
                            break;
                        }
                    }
                    if (gridFull)
                        break;
                }

            }
        }

        calculateGrid(x, y);       
    }

    void Map::calculateGrid(double x, double y) {
        std::vector<std::vector<int>> copy1;
        copy1 = grid;

        wavePropagation(grid, x, y);

        for (int g = 0; g < copy1.size(); g++) {        //copy1 = obstacle transform
            for (int h = 0; h < copy1[g].size(); h++) {
                if (copy1[g][h] != -2) {
                    if (g == 0 || h == 0 || g == copy1.size() - 1 || h == copy1[0].size() - 1)
                        copy1[g][h] = 30;
                    else if (copy1[g - 1][h - 1] == -2 || copy1[g - 1][h] == -2 || copy1[g - 1][h + 1] == -2 || copy1[g][h - 1] == -2
                        || copy1[g][h + 1] == -2 || copy1[g + 1][h - 1] == -2 || copy1[g + 1][h] == -2 || copy1[g + 1][h + 1] == -2)
                        copy1[g][h] = 30;
                    else
                        copy1[g][h] = 10;
                }
            }
        }
        obstacleTransformGrid = copy1;

        std::cout << "grid pomo: " << std::endl;
        for (int g = 0; g < copy1.size(); g++) {
            for (int h = 0; h < copy1[g].size(); h++) {
                //if (grid[g][h] == -2)
                    //std::cout << "1 ";
                //else
                std::cout << copy1[g][h] << " ";
            }
            std::cout << std::endl;
        }
    }

    void Map::setArenaSize(int x, int y) {
        arenaX = x;
        arenaY = y;
    }

    int Map::getDispX() {
        return displacementX;
    }

    int Map::getDispY() {
        return displacementY;
    }

    int Map::getArenaX() {
        return arenaX;
    }

    int Map::getArenaY() {
        return arenaY;
    }

    int* Map::getCurrentCell(double positionX, double positionY) {
        int poseX = round(positionX * 100) + arenaX / 2;        // mo¿e byæ do poprawy map[0].size() / 2;
        int poseY = -round(positionY * 100) + arenaY / 2;           // pozycja robota na mapie
        //std::cout << "positionX = " << positionX << "   poseX = " << poseX << std::endl;
        //std::cout << "positionY = " << positionY << "   poseY = " << poseY << std::endl;
        poseX = (poseX - 5 - displacementX) / 35;
        if (poseX >= gridSizeX)
            poseX = gridSizeX - 1;
        poseY = (poseY - 5 - displacementY) / 35;      //!!!!! niejednolite rozmiary
        if (poseY >= gridSizeY)
            poseY = gridSizeY - 1;
        //std::cout << "xxxddd " << poseX << "    yyyyyyyy " << poseY << std::endl;
        int* currentCell = new int[2];
        currentCell[0] = poseX;
        currentCell[1] = poseY;
        //std::cout << "currX = " << currentCell[0] << "  , currY = " << currentCell[1] << std::endl;
        return currentCell;
    }

    void Map::wavePropagation(std::vector <std::vector<int>> &grid, double positionX, double positionY) {
        int* currentCell = getCurrentCell(positionX, positionY);
        
        //std::cout << "currX = " << currentCell[0] << "  , currY = " << currentCell[1] << std::endl;
        grid[currentCell[1]][currentCell[0]] = 0;
        delete currentCell;
        bool isFound = true;
        int i = 0;
        while (isFound) {
            isFound = false;
            for (int g = 0; g < grid.size(); g++) {
                for (int h = 0; h < grid[g].size(); h++) {
                    if (grid[g][h] == i) {
                        isFound = true;
                        if (g > 0 && grid[g - 1][h] == -1)
                            grid[g - 1][h] = i + 1;
                        if (h > 0 && grid[g][h - 1] == -1)
                            grid[g][h - 1] = i + 1;
                        if (g < grid.size() - 1 && grid[g + 1][h] == -1)
                            grid[g + 1][h] = i + 1;
                        if (h < grid[g].size() - 1 && grid[g][h + 1] == -1)
                            grid[g][h + 1] = i + 1;
                    }
                }
            }
            i++;
        }
        for (int g = 0; g < grid.size(); g++) {
            for (int h = 0; h < grid[g].size(); h++) {
                if (grid[g][h] == -1)
                    grid[g][h] = -2;
            }
        }
    }

    bool Map::areNeighbourCellsOccupied(double x, double y) {
        int* currentCell = getCurrentCell(x, y);
        if (currentCell[0] > 0 && grid[currentCell[1]][currentCell[0] - 1] > 0)      // zachód
            return false;
        if (currentCell[1] > 0 && grid[currentCell[1] - 1][currentCell[0]] > 0)  // pó³noc
            return false;
        if (currentCell[0] < grid[0].size() - 1 && grid[currentCell[1]][currentCell[0] + 1] > 0)    // wschód
            return false;        
        if (currentCell[1] < grid.size() - 1 && grid[currentCell[1] + 1][currentCell[0]] > 0)   // po³udnie
            return false;

        return true;
    }

    void Map::setGridCell(int x, int y, int val) {
        grid[y][x] = val;
    }

    void Map::setGrid(std::vector <std::vector<int>> newGrid) {
        grid = newGrid;
    }

    std::vector <std::vector<int>> Map::getObsTransformGrid() {
        return obstacleTransformGrid;
    }

    std::vector <std::vector<double>> Map::getPoints() {
        return points;
    }

    std::vector <std::vector<bool>> Map::getMap() {
        return map;
    }

    std::vector <std::vector<int>> Map::getGrid() {
        return grid;
    }

