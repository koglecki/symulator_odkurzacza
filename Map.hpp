#ifndef Map_hpp
#define Map_hpp
#include <vector>

class Map {
    bool mapping = false;       //czy aktualnie trwa tworzenie mapy
    double mapClosurePosition[2] = { 0, 0 };        //wspó³rzêdne zamkniêcia mapy
    bool mapOpened = false;                //czy mapa jest otwarta (czy mo¿na j¹ zamkn¹æ)
    bool wallFound = false;            //czy pierwsza œciana zosta³a znaleziona
    bool firstTurn = true;              //czy trwa pierwszy obrót podczas mapowania
    std::vector <std::vector<double>> map;

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
    void printMap();
    std::vector <std::vector<double>> getMap();

};
#endif