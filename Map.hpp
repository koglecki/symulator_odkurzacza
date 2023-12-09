#ifndef Map_hpp
#define Map_hpp
#include <vector>

class Map {
private:
    bool mapping = false;       //czy aktualnie trwa tworzenie mapy
    double mapClosurePosition[2] = { 0, 0 };        //wsp�rz�dne zamkni�cia mapy
    bool mapOpened = false;                //czy mapa jest otwarta (czy mo�na j� zamkn��)
    bool wallFound = false;            //czy pierwsza �ciana zosta�a znaleziona
    bool firstRotation = true;              //czy trwa pierwszy obr�t podczas mapowania
    int mapCorrectionValue = 50;
    int gridSizeX;
    int gridSizeY;
    int displacementX = 0;
    int displacementY = 0;
    bool** globalMap;
    std::vector <std::vector<double>> points;
    std::vector <std::vector<bool>> map;
    std::vector <std::vector<int>> grid;
    std::vector <std::vector<int>> obstacleTransformGrid;
    int arenaX = 400;
    int arenaY = 400;
    double obsClosurePosition[2] = { -1000, 0 };
    bool obsOpened = false;
    bool obstacling = false;

    void calculateWavePropagation(std::vector <std::vector<int>>& grid);

public:
    void beginMapping();

    void finishMapping();

    void beginObstacling();

    void finishObstacling();

    bool isMapping();

    bool isObstacling();

    bool isWallFound();

    void setWallFound();

    void setMapCorrectionValue(int value);

    double* getMapClosurePosition();

    double* getObsClosurePosition();

    void setObsClosurePosition(double x, double y);

    void openMap();

    void closeMap();

    void openObs();

    void closeObs();

    bool isObsOpened();

    int getDispX();

    int getDispY();

    int getArenaX();

    int getArenaY();

    void setArenaSize(int x, int y);

    bool isMapOpened();

    void setMapClosurePosition(double x, double y);

    bool isFirstRotation();

    void finishFirstRotation();

    bool isPoint(double x, double y);

    void insertPoint(double x, double y);

    void newGlobalMap();

    void createMap(double x, double y, bool newMap);

    void optimizeMap();

    bool areNeighbourCellsOccupied(double x, double y);

    void createGrid(bool newGrid);

    void calculateObstacleTransformGrid();

    void wavePropagation(std::vector <std::vector<int>>& grid, double positionX, double positionY);

    void fastWavePropagation(std::vector <std::vector<int>>& grid);

    std::vector <std::vector<double>> getPoints();

    std::vector <std::vector<bool>> getMap();

    std::vector <std::vector<int>> getGrid();

    std::vector <std::vector<int>> getObsTransformGrid();

    void printGrid();

    void setGridCell(int x, int y, int val);

    int* getCurrentCell(double positionX, double positionY);

    ~Map();
};
#endif