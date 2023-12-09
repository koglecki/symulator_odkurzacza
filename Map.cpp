#include "Map.hpp"
#include <vector>
#include <iostream>

    void Map::beginMapping() {
        mapping = true;
    }

    void Map::finishMapping() {
        mapping = false;
    }

    void Map::beginObstacling() {
        obstacling = true;
    }

    void Map::finishObstacling() {
        obstacling = false;
    }

    bool Map::isMapping() {
        return mapping;
    }

    bool Map::isObstacling() {
        return obstacling;
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

    double* Map::getObsClosurePosition() {
        return obsClosurePosition;
    }

    void Map::openMap() {
        mapOpened = true;
    }

    void Map::closeMap() {
        mapOpened = false;
    }

    void Map::openObs() {
        obsOpened = true;
    }

    void Map::closeObs() {
        obsOpened = false;
    }

    bool Map::isMapOpened() {
        return mapOpened;
    }

    bool Map::isObsOpened() {
        return obsOpened;
    }

    void Map::setMapClosurePosition(double x, double y) {
        mapClosurePosition[0] = x;
        mapClosurePosition[1] = y;
    }

    void Map::setObsClosurePosition(double x, double y) {
        obsClosurePosition[0] = x;
        obsClosurePosition[1] = y;
    }

    bool Map::isFirstRotation() {
        return firstRotation;
    }

    void Map::finishFirstRotation() {
        firstRotation = false;
    }

    void Map::setMapCorrectionValue(int value) {
        mapCorrectionValue = value;
    }

    void Map::setGridCell(int x, int y, int val) {
        grid[y][x] = val;
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

    void Map::newGlobalMap() {
        globalMap = new bool* [arenaY];     // globalna mapa zajêtoœci
        for (int i = 0; i < arenaY; i++)
            globalMap[i] = new bool[arenaX];
        for (int i = 0; i < arenaY; i++) {
            for (int j = 0; j < arenaX; j++)
                globalMap[i][j] = false;
        }
    }

    void Map::createMap(double x, double y, bool newMap) {     // tworzenie mapy (siatki zajêtoœci), true = przeszkoda
        if (newMap)
            newGlobalMap();
             
        for (int j = 0; j < points.size(); j++) {           // przypisanie punktów z listy do globalnej mapy
            int coordY = -round(points[j][1] * 100) + arenaY / 2;
            int coordX = round(points[j][0] * 100) + arenaX / 2;
            
            if (coordY >= arenaY)
                globalMap[arenaY - 1][coordX] = true;
            else if (coordY < 0)
                globalMap[0][coordX] = true;
            else if (coordX >= arenaX)
                globalMap[coordY][arenaX - 1] = true;            
            else if (coordX < 0)
                globalMap[coordY][0] = true;
            else
                globalMap[coordY][coordX] = true;
        }
        points.clear();
        optimizeMap();

        if (newMap) {
            createGrid(true);
            calculateObstacleTransformGrid();
            wavePropagation(grid, x, y);
        }
        else {
            createGrid(false);
            calculateObstacleTransformGrid();
            fastWavePropagation(grid);
        }
    }

    void Map::optimizeMap() {       // usuwanie niepotrzebnych obszarów na mapie
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

    void Map::createGrid(bool newGrid) {
        int xsize = (map[0].size() - 5) / 35;       // 5 to odleg³oœæ skrajnie lewej kratki od lewej œciany
        int ysize = (map.size() - 5) / 35;          // 35 to rozmiar kratki       
        gridSizeX = xsize;
        gridSizeY = ysize;
        
        if (newGrid) {
            std::vector<std::vector<int>> v(ysize, std::vector<int>(xsize, -1));
            grid = v;
        }
        std::vector<std::vector<int>> copy;
        copy = grid;

        bool isGridOccupied;
        for (int g = 0; g < grid.size(); g++) {         // -1 kratka jest wolna
            for (int h = 0; h < grid[g].size(); h++) {  // -2 kratka zajêta
                isGridOccupied = false;
                for (int i = 5 + g * 35; i < 40 + g * 35; i++) {
                    for (int j = 5 + h * 35; j < 40 + h * 35; j++) {
                        if (map[i][j]) {
                            grid[g][h] = -2;
                            isGridOccupied = true;
                            break;
                        }
                    }
                    if (isGridOccupied)
                        break;
                }

            }
        }

        if (!newGrid) {         // aktualizacja siatki o nowe przeszkody
            for (int g = 0; g < copy.size(); g++) {
                for (int h = 0; h < copy[g].size(); h++) {
                    if (grid[g][h] == -2)
                        copy[g][h] = -2;
                }
            }
            grid = copy;
        }
    }

    void Map::calculateObstacleTransformGrid() {
        obstacleTransformGrid = grid;

        for (int g = 0; g < obstacleTransformGrid.size(); g++) {
            for (int h = 0; h < obstacleTransformGrid[g].size(); h++) {
                if (obstacleTransformGrid[g][h] != -2) {
                    if (g == 0 || h == 0 || g == obstacleTransformGrid.size() - 1 || h == obstacleTransformGrid[0].size() - 1)
                        obstacleTransformGrid[g][h] = 30;
                    else if (obstacleTransformGrid[g - 1][h - 1] == -2 || obstacleTransformGrid[g - 1][h] == -2
                        || obstacleTransformGrid[g - 1][h + 1] == -2 || obstacleTransformGrid[g][h - 1] == -2
                        || obstacleTransformGrid[g][h + 1] == -2 || obstacleTransformGrid[g + 1][h - 1] == -2
                        || obstacleTransformGrid[g + 1][h] == -2 || obstacleTransformGrid[g + 1][h + 1] == -2)
                        obstacleTransformGrid[g][h] = 30;
                    else
                        obstacleTransformGrid[g][h] = 10;
                }
            }
        }
    }

    int* Map::getCurrentCell(double positionX, double positionY) {
        int poseX = round(positionX * 100) + arenaX / 2;
        int poseY = -round(positionY * 100) + arenaY / 2;           // pozycja robota na mapie
        
        poseX = (poseX - 5 - displacementX) / 35;
        if (poseX >= gridSizeX)
            poseX = gridSizeX - 1;
        poseY = (poseY - 5 - displacementY) / 35;
        if (poseY >= gridSizeY)
            poseY = gridSizeY - 1;
        
        int* currentCell = new int[2];
        currentCell[0] = poseX;
        currentCell[1] = poseY;
    
        return currentCell;
    }

    void Map::calculateWavePropagation(std::vector <std::vector<int>>& grid) {
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

    void Map::wavePropagation(std::vector <std::vector<int>> &grid, double positionX, double positionY) {
        int* currentCell = getCurrentCell(positionX, positionY);
        grid[currentCell[1]][currentCell[0]] = 0;
        delete currentCell;

        calculateWavePropagation(grid);
    }

    void Map::fastWavePropagation(std::vector <std::vector<int>>& grid) {
        int x = 0;
        int y = 0;
        bool isFound = false;

        for (int g = 0; g < grid.size(); g++) {         // losowy punkt startowy fali, który nie jest przeszkod¹
            for (int h = 0; h < grid[g].size(); h++) {
                if (grid[g][h] == -1) {
                    x = h;
                    y = g;
                    isFound = true;
                    break;
                }
            }
            if (isFound)
                break;
        }

        std::vector <std::vector<int>> copy = grid;
        for (int g = 0; g < copy.size(); g++) {         // powrót do reprezentacji zajêtoœci przez przeszkody
            for (int h = 0; h < copy[g].size(); h++) {
                if (copy[g][h] != -2)
                    copy[g][h] = -1;
            }
        }
        copy[y][x] = 0;

        calculateWavePropagation(copy);

        for (int g = 0; g < grid.size(); g++) {
            for (int h = 0; h < grid[g].size(); h++) {
                if (copy[g][h] == -2)
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
        delete currentCell;
        return true;
    }

    void Map::printGrid() {
        for (int g = 0; g < grid.size(); g++) {
            for (int h = 0; h < grid[g].size(); h++) {
                std::cout << grid[g][h] << " ";
            }
            std::cout << std::endl;
        }
    }

    Map::~Map() {
        for (int i = 0; i < arenaY; i++)
            delete [] globalMap[i];
        delete [] globalMap;
    }