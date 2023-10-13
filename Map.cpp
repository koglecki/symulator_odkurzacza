#include "Map.hpp"

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
    void Map::insertPoint(double x, double y) {
        map[x, y] = 1;
    }
    std::map <double, double> Map::getMap() {
        return map;
    }

