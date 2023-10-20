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
    void Map::createMap() {     // tworzenie mapy (siatki zajêtoœci), true = przeszkoda
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
        createGrid();
    }

    void Map::optimizeMap() {
        int counter = 0;
        for (int i = 0; i < map.size(); i++) {
            for (int j = 0; j < map[i].size(); j++) {
                if (!map[i][j])
                    counter++;
                else if (map[i][j] && counter <= 50) {
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
                else if (map[j][i] && counter <= 50) {
                    for (int k = 1; k <= counter; k++)
                        map[j - k][i] = true;
                    counter = 0;
                }
                else
                    counter = 0;
            }
        }
    }

    void Map::createGrid() {
        int xsize = (map[0].size() - 20) / 35;
        int ysize = (map.size() - 20) / 35;

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
        calculateGrid();       
    }

    void Map::calculateGrid() {
        int poseX = 0;
        int poseY = 0;

        grid[poseY][poseX] = 0;
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

