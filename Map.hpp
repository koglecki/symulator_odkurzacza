#ifndef Map_hpp
#define Map_hpp
#include <vector>

class Map {
private:
    bool mapping = false;       //czy aktualnie trwa tworzenie mapy
    double mapClosurePosition[2] = { 0, 0 };        //wspó³rzêdne zamkniêcia mapy
    bool mapOpened = false;                //czy mapa jest otwarta (czy mo¿na j¹ zamkn¹æ)
    bool wallFound = false;            //czy pierwsza œciana zosta³a znaleziona
    bool firstTurn = true;              //czy trwa pierwszy obrót podczas mapowania
    std::vector <std::vector<double>> points;
    std::vector <std::vector<bool>> map;
    std::vector <std::vector<bool>> grid;

public:
    void beginMapping();
    void finishMapping();
    bool isMapping();
    bool isWallFound();
    void setWallFound();
    double* getMapClosurePosition();
    void openMap();
    void closeMap();
    bool isMapOpened();
    void setMapClosurePosition(double x, double y);
    bool isFirstTurn();
    void finishFirstTurn();
    bool isPoint(double x, double y);
    void insertPoint(double x, double y);
    void createMap();
    void optimizeMap();
    void createGrid();
    std::vector <std::vector<double>> getPoints();
    std::vector <std::vector<bool>> getMap();
    std::vector <std::vector<bool>> getGrid();
};
#endif