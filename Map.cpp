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
    }

    std::vector <std::vector<double>> Map::getPoints() {
        return points;
    }

    std::vector <std::vector<bool>> Map::getMap() {
        return map;
    }

