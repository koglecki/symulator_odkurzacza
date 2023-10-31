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
        
        int xsize = int(maxX - minX);
        int ysize = int(maxY - minY);
        std::vector<std::vector<bool>> v(ysize, std::vector<bool>(xsize, false));
        map = v;
        
        for (int j = 0; j < points.size(); j++) {
            if (int(-round(points[j][1] * 100) + maxY) == ysize)
                map[-round(points[j][1] * 100) + maxY - 1][round(points[j][0] * 100) - minX] = true;
            else if (int(round(points[j][0] * 100) - minX) == xsize)
                map[-round(points[j][1] * 100) + maxY][round(points[j][0] * 100) - minX - 1] = true;
            else
                map[-round(points[j][1] * 100) + maxY][round(points[j][0] * 100) - minX] = true;
        }

        optimizeMap();
        createGrid(x, y);
    }

    void Map::setMapCorrectionValue(int value) {
        mapCorrectionValue = value;
    }

    void Map::optimizeMap() {
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
        }
        counter = 0;
        for (int i = 0; i < map[0].size(); i++) {
            for (int j = 0; j < map.size(); j++) {
                if (!map[j][i])
                    counter++;
                else if (map[j][i] && counter <= mapCorrectionValue) {
                    for (int k = 1; k <= counter; k++) {
                        if (j - k >= 0)
                            map[j - k][i] = true;
                    }
                    counter = 0;
                }
                else
                    counter = 0;
            }
        }
    }

    void Map::createGrid(double x, double y) {
        int xsize = (map[0].size() - 10) / 35;
        if (double((map[0].size() - 10) / 35) - xsize < 0.15)
            xsize -= 1;
        int ysize = (map.size() - 10) / 35;
        if (double((map.size() - 10) / 35) - ysize < 0.15)
            ysize -= 1;
        gridSizeX = xsize;
        gridSizeY = ysize;

        std::vector<std::vector<int>> v(ysize, std::vector<int>(xsize, -1));
        grid = v;

        bool gridFull;
        for (int g = 0; g < grid.size(); g++) {
            for (int h = 0; h < grid[g].size(); h++) {
                gridFull = false;
                for (int i = 10 + g * 35; i < 45 + g * 35; i++) {
                    for (int j = 10 + h * 35; j < 45 + h * 35; j++) {
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

        std::vector<std::vector<int>> copy2;
        copy2 = copy1;

        std::vector<std::vector<int>> copy3;
        copy3 = grid;

        std::vector<std::vector<int>> open;
        open = grid;      //-1,  -2 tam gdzie przeszkody
        for (int g = 0; g < open.size(); g++) {
            for (int h = 0; h < open[g].size(); h++) {
                if (open[g][h] != -2 && open[g][h] != 0)
                    open[g][h] = -1;
            }
        }

        //4 0
        int nr = 0;
        int cost = -1;
        int mincost = -1;
        open[4][0] = -3;    //-3 otw, -4 zam
        if (4 != 0 && open[4 - 1][0] != -2) {
            open[4 - 1][0] = -3;
            nr = 1;
            cost = grid[4 - 1][0] + copy1[4 - 1][0];
            mincost = cost;
        }
        if (4 != open.size() - 1 && open[4 + 1][0] != -2) {
            open[4 + 1][0] = -3;
            cost = grid[4 + 1][0] + copy1[4 + 1][0];
            if (mincost == -1 || cost < mincost) {
                mincost = cost;
                nr = 2;
            }
        }
        if (0 != 0 && open[4][0 - 1] != -2) {
            open[4][0 - 1] = -3;
            cost = grid[4][0-1] + copy1[4 ][0-1];
            if (mincost == -1 || cost < mincost) {
                mincost = cost;
                nr = 3;
            }
        }
        if (0 != open[0].size() && open[4][0 + 1] != -2) {
            open[4][0 + 1] = -3;
            cost = grid[4][0+1] + copy1[4][0+1];
            if (mincost == -1 || cost < mincost) {
                mincost = cost;
                nr = 4;
            }
        }

        //std::cout << "numer " << nr << "wartosc " << mincost << std::endl;
        
    }

    int* Map::getCurrentCell(double positionX, double positionY) {
        int poseX = round(positionX * 100) + map[0].size() / 2;        // mo¿e byæ do poprawy
        int poseY = -round(positionY * 100) + map.size() / 2;
        
        poseX = (poseX - 10) / 35;
        if (poseX >= gridSizeX)
            poseX = gridSizeX - 1;
        poseY = (poseY - 10) / 35;
        if (poseY >= gridSizeY)
            poseY = gridSizeY - 1;
        //std::cout << "xxxddd " << poseX << "    yyyyyyyy " << poseY << std::endl;
        int currentCell[2] = { poseX, poseY };
        return currentCell;
    }

    void Map::wavePropagation(std::vector <std::vector<int>> &grid, double positionX, double positionY) {
        int* currentCell = getCurrentCell(positionX, positionY);
        
        grid[currentCell[1]][currentCell[0]] = 0;
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

