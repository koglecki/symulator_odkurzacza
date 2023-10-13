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
        for (int i = 0; i < map.size(); i++)
        {
            if (map[i][0] == x && map[i][1] == y)
                return true;
        }
        return false;
    }
    void Map::insertPoint(double x, double y) {
        if (!isPoint(x, y))
            map.push_back({x, y});
    }
    void Map::printMap() {
        for (int i = 0; i < map.size(); i++)
        {
            std::cout << i + 1 << " -> " << map[i][0] << "  " << map[i][1] << std::endl;
        }
    }
    std::vector <std::vector<double>> Map::getMap() {
        return map;
    }

