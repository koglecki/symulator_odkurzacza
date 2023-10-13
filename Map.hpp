#ifndef Map_hpp
#define Map_hpp
#include <map>

class Map {
    bool mapping = false;       //czy aktualnie trwa tworzenie mapy
    double mapClosurePosition[2] = { 0, 0 };        //wspó³rzêdne zamkniêcia mapy
    bool mapOpened = false;                //czy mapa jest otwarta (czy mo¿na j¹ zamkn¹æ)
    bool wallFound = false;            //czy pierwsza œciana zosta³a znaleziona
    bool firstTurn = true;              //czy trwa pierwszy obrót podczas mapowania
    std::map <double, double> map;

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
    void insertPoint(double x, double y);
    std::map <double, double> getMap();

};
#endif