#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Lidar.hpp>
#include <webots/Display.hpp>
#include <webots/Pen.hpp>
#include <iostream>
#include <cmath>
#include <queue>
//#include <map>
#include <vector>

using namespace webots;
const double pi = 3.14159265358979323846;
//kratka = 0.34m
//ca³oœæ = 0.68 x 0.68 m
/*
int map[6][8] = { {1, 1, 1, 1, 1, 1, 1, 1}, {1, 0, 0, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 0, 0, 1}, {1, 1, 1, 1, 1, 1, 1, 1} };

bool isEveryCellVisited() {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 8; j++) {
            if (map[i][j] == 0)
                return false;
        }
    }
    return true;
}

bool isVisited(int x, int y) {

}

void spanningTree(int x, int y) {   //x = 4, y = 4
    std::vector <int[2]> freeCells;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 8; j++) {
            if (map[i][j] == 0)
                freeCells.push_back({i, j});
        }
    }
    std::queue <int[2]> tree;
    std::queue <int[2]> neighbor;
    int current[2] = {x, y};
    tree.push(current);

    while (!isEveryCellVisited()) {
        if (current[1] > 0) {
            if (map[current[0]][current[1] - 1] == 0)
                neighbor.push({ current[0] , current[1] - 1 });
        }
        if (current[0] > 0) {
            if (map[current[0] - 1][current[1]] == 0)
                neighbor.push({ current[0] - 1 , current[1] });
        }
        if (current[1] < 7) {
            if (map[current[0]][current[1] + 1] == 0)
                neighbor.push({ current[0] , current[1] + 1 });
        }
        if (current[0] < 5) {
            if (map[current[0] + 1][current[1]] == 0)
                neighbor.push({ current[0] + 1 , current[1] });
        }


    }

}
*/
class Map {
    bool mapping = false;       //czy aktualnie trwa tworzenie mapy
    double mapClosurePosition[2] = { 0, 0 };        //wspó³rzêdne zamkniêcia mapy
    bool mapOpened = false;                //czy mapa jest otwarta (czy mo¿na j¹ zamkn¹æ)
    bool wallFound = false;            //czy pierwsza œciana zosta³a znaleziona
    bool firstTurn = true;              //czy trwa pierwszy obrót podczas mapowania
    //std::map <double, double> map;

public:
    void beginMapping() {
        mapping = true;
    }
    void finishMapping() {
        mapping = false;
    }
    bool isMapping() {
        return mapping;
    }
    bool isWallFound() {
        return wallFound;
    }
    void setWallFound() {
        wallFound = true;
    }
    double* getMapClosurePosition() {
        return mapClosurePosition;
    }
    void openMap() {
        mapOpened = true;
    }
    void closeMap() {
        mapOpened = false;
    }
    bool isMapOpened() {
        return mapOpened;
    }
    void setMapClosurePosition(double x, double y) {
        mapClosurePosition[0] = x;
        mapClosurePosition[1] = y;
    }
    bool isFirstTurn() {
        return firstTurn;
    }
    void finishFirstTurn() {
        firstTurn = false;
    }
    void insertPoint(double x, double y) {
        //map[x, y] = 1;
    }
    //std::map <double, double> getMap() {
        //return map;
    //}

};

class CleaningRobot {
private:
    const double wheelRadius = 0.04;
    const double wheelbase = 0.26;
    const double robotRadius = 0.169;
    int timeStep;
    double poseSensor[2] = { 0, 0 };         //k¹t przebyty przez lewe i prawe ko³o wed³ug czujników (radiany)
    double prevPoseSensor[2] = { 0, 0 };     //poprzedni pomiar k¹ta
    double position[3] = { 0.51, 1.7, 3.14159 };      //aktualna pozycja robota (x,y,theta)
    int mode = 1;                   //tryb pracy robota
    int displayWidth;
    int displayHeight;

    void clearRobotDisplay() {
        display->setColor(0x000000);
        display->drawOval(position[0] * 100 + displayWidth / 2, -position[1] * 100 + displayHeight / 2, 15, 15);   //usuniêcie starej pozycji robota z ekranu
        display->fillOval(position[0] * 100 + displayWidth / 2, -position[1] * 100 + displayHeight / 2, 15, 15);
    }

public:
    Robot* robot = new Robot();
    Motor* right_motor = robot->getMotor("right_motor");
    Motor* left_motor = robot->getMotor("left_motor");
    PositionSensor* right_position = robot->getPositionSensor("right_position");
    PositionSensor* left_position = robot->getPositionSensor("left_position");
    Lidar* lidar = robot->getLidar("lidar");
    Display* display = robot->getDisplay("display");
    Pen* pen = robot->getPen("pen");

    CleaningRobot() {
        timeStep = (int)robot->getBasicTimeStep();
        right_motor->setPosition(INFINITY);
        right_motor->setVelocity(0.0);
        left_motor->setPosition(INFINITY);
        left_motor->setVelocity(0.0);
        right_position->enable(timeStep);
        left_position->enable(timeStep);
        lidar->enable(timeStep);
        lidar->enablePointCloud();
        pen->write(1);
        displayWidth = display->getWidth();
        displayHeight = display->getHeight();
    }

    void setMode(int m) {
        mode = m;
    }

    int getMode() {
        return mode;
    }

    double* getPosition() {
        return position;
    }

    double* getPrevPoseSensor() {
        return prevPoseSensor;
    }

    double* getPoseSensor() {
        return poseSensor;
    }

    int getTimeStep() {
        return timeStep;
    }

    void setDriveParameters(double leftVoltage, double rightVoltage) {
        double leftRotatingSpeed = leftVoltage * 3; // rad/s      max 5V
        double rightRotatingSpeed = rightVoltage * 3; // 1 obrót = 6.2832 rad = 2 * pi
        double vleft = leftRotatingSpeed * wheelRadius; // m/s (prêdkoœæ lewego ko³a)
        double vright = rightRotatingSpeed * wheelRadius; // m/s (prêdkoœæ prawego ko³a)
        //std::cout << "vleft = " << vleft << " m/s  , vright = " << vright << " m/s" << std::endl;

        right_motor->setVelocity(rightRotatingSpeed);
        left_motor->setVelocity(leftRotatingSpeed);
    }

    const float* calculatePosition() {
        poseSensor[0] = left_position->getValue();
        poseSensor[1] = right_position->getValue();        //pobranie wartoœci z czujnika pozycji
        const float* rangeImage = lidar->getRangeImage();

        double sright = (poseSensor[1] - prevPoseSensor[1]) * wheelRadius;    // droga prawego ko³a
        double sleft = (poseSensor[0] - prevPoseSensor[0]) * wheelRadius;     // droga lewego ko³a

        double deltatheta = (sright - sleft) / wheelbase; // zmiana k¹ta
        deltatheta -= 0.00945 * deltatheta;
        double s = (sleft + sright) / 2; // droga ogólna
        //std::cout << s/(timeStep/1000) << std::endl;

        double xd[3] = { s * cos(position[2]), s * sin(position[2]), deltatheta }; // macierz przekszta³cenia
        //std::cout << xd[0] << " " << xd[1] << " " << xd[2] << std::endl;
        double pp[3] = { position[0] + xd[0], position[1] + xd[1], position[2] + xd[2] }; // nowa pozycja

        clearRobotDisplay();
        position[0] = pp[0];
        position[1] = pp[1];
        position[2] = pp[2];

        std::cout << "Pozycja:   x = " << position[0] << " , y = " << position[1] << " , theta = " << position[2] << std::endl;
        return rangeImage;
    }

    void refreshDisplay(const float* rangeImage) {
        display->setColor(0xff0000);
        display->drawOval(position[0] * 100 + displayWidth / 2, -position[1] * 100 + displayHeight / 2, 15, 15);   //nowa pozycja robota na ekranie
        display->fillOval(position[0] * 100 + displayWidth / 2, -position[1] * 100 + displayHeight / 2, 15, 15);

        int higher = 2;
        int lower = -2;
        for (int i = 0; i < lidar->getHorizontalResolution(); i++) {     //wizualizacja odczytów lidara
            if (*(rangeImage + i) < 1)
                display->setColor(0xFF00FF);
            else
                display->setColor(0x000000);
            double L = *(rangeImage + i) * 100; // odleg³oœæ od sensora
            double x3 = position[0] * 100 + displayWidth / 2 + (17 * cos(position[2]));
            double y3 = -position[1] * 100 + displayHeight / 2 - (17 * sin(position[2]));

            double x4 = x3 + (L * cos(position[2] + pi / 2 - i * (pi / (lidar->getHorizontalResolution() - 1)))); // 3.14/199 = 0.015786
            double y4 = y3 - (L * sin(position[2] + pi / 2 - i * (pi / (lidar->getHorizontalResolution() - 1))));  // -1 bo jest 199 odcinków miêdzy punktami

            if (*(rangeImage + i) < 1) {
                if ((lidar->getHorizontalResolution() - y4) / 100 < position[1] + 0.174 && -(y4 - lidar->getHorizontalResolution()) / 100 > position[1] - 0.174 && higher == 2) //górne pkt
                    higher = i;
                if (-(y4 - lidar->getHorizontalResolution()) / 100 < position[1] - 0.174 && lower == -2) //dolne pkt
                    lower = i;
            }
            //std::cout << position[2] + pi / 2 - i * (pi / (lidar->getHorizontalResolution() - 1)) << " ";
            //std::cout << *(rangeImage + i) << " ";
            //std::cout << "x = " << (x4 - 275) / 100 << " ";
            //std::cout << "y = " << -(y4 - 275) / 100 << " ";
            display->drawPixel(round(x4), round(y4));
        }
        //std::cout << "higher = " << higher << " , lower = " << lower << std::endl;

        //return higher, lower;
    }

    void refreshSensorValues() {
        prevPoseSensor[0] = poseSensor[0];
        prevPoseSensor[1] = poseSensor[1];
    }

    /*double* calculatePoint(double distance, int i) {
        double x2 = position[0] + (robotRadius * cos(position[2]));
        double y2 = position[1] + (robotRadius * sin(position[2]));

        double x3 = x2 + (distance * cos(position[2] + pi / 2 - i * (pi / (lidar->getHorizontalResolution() - 1))));
        double y3 = y2 + (distance * sin(position[2] + pi / 2 - i * (pi / (lidar->getHorizontalResolution() - 1))));

        double xy[2] = { x3, y3 };
        return xy;
    }*/

    void turnRobot(double startAngle, double angle) {      //obrót o konkretny k¹t
        if ((angle >= 0 && position[2] > startAngle + angle) || (angle < 0 && position[2] < startAngle + angle)) {
            stopRobot();                                    //zatrzymywanie robota po obrocie
            if (poseSensor[0] - prevPoseSensor[0] == 0) {
                mode = 2;

                if (position[2] >= pi * 2)
                    position[2] = position[2] - pi * 2;
                else if (position[2] < 0)
                    position[2] = position[2] + pi * 2;
            }
        }
        else if ((angle >= 0 && position[2] <= startAngle + angle - 0.3) || (angle < 0 && position[2] >= startAngle + angle + 0.3)) {
            if (angle >= 0)                                 //pe³na prêdkoœæ obrotu
                setDriveParameters(-1, 1);
            else
                setDriveParameters(1, -1);
        }
        else if ((angle >= 0 && position[2] > startAngle + angle - 0.3) || (angle < 0 && position[2] < startAngle + angle + 0.3)) {
            if (angle >= 0)                                 //zmniejszona prêdkoœæ obrotu
                setDriveParameters(-0.106, 0.106);
            else
                setDriveParameters(0.106, -0.106);
        }
    }

    void turnRobot2(double startAngle, double angle) {
        if ((angle >= 0 && position[2] > startAngle + angle) || (angle < 0 && position[2] < startAngle + angle)) {
            stopRobot();                                    //zatrzymywanie robota po obrocie
            if (poseSensor[0] - prevPoseSensor[0] == 0) {
                mode = 12;

                if (position[2] >= pi * 2)
                    position[2] = position[2] - pi * 2;
                else if (position[2] < 0)
                    position[2] = position[2] + pi * 2;
            }
        }
        else if ((angle >= 0 && position[2] <= startAngle + angle - 0.3) || (angle < 0 && position[2] >= startAngle + angle + 0.3)) {
            if (angle >= 0)                                 //pe³na prêdkoœæ obrotu
                setDriveParameters(-1, 1);
            else
                setDriveParameters(1, -1);
        }
        else if ((angle >= 0 && position[2] > startAngle + angle - 0.3) || (angle < 0 && position[2] < startAngle + angle + 0.3)) {
            if (angle >= 0)                                 //zmniejszona prêdkoœæ obrotu
                setDriveParameters(-0.106, 0.106);
            else
                setDriveParameters(0.106, -0.106);
        }
    }

    void driveRobot(double voltage) {       // jazda prosto z jedn¹ nastaw¹
        setDriveParameters(voltage, voltage);
    }

    double driveRobotByDistance(double voltage, double distance) {
        double s = 0;
        if (distance > 0) {
            double sright = (poseSensor[1] - prevPoseSensor[1]) * wheelRadius;
            double sleft = (poseSensor[0] - prevPoseSensor[0]) * wheelRadius;
            s = (sleft + sright) / 2;
            setDriveParameters(voltage, voltage);
        }
        else
            stopRobot();
        return s;
    }

    void stopRobot() {                      //zatrzymanie robota
        setDriveParameters(0, 0);
        if (getPoseSensor()[0] - getPrevPoseSensor()[0] == 0)
            if (mode == 11)
                mode = 10;
            else
                mode = 4;
    }

    void stopTurningRobot() {                      //zatrzymanie obrotu robota
        setDriveParameters(0, 0);
        if (getPoseSensor()[0] - getPrevPoseSensor()[0] == 0)
            mode = 2;
    }

    ~CleaningRobot() {
        delete robot, right_motor, left_motor, right_position, left_position, lidar, display, pen;
    }
};

int main(int argc, char** argv) {
    CleaningRobot* cr = new CleaningRobot();
    Map* map = new Map();
    double startAngle = cr->getPosition()[2];
    double distance = 0;
    double d;


    while (cr->robot->step(cr->getTimeStep()) != -1) {      //g³ówna pêtla programu
        std::cout << "mode = " << cr->getMode() << std::endl;

        const float* rangeImage = cr->calculatePosition();
        std::cout << *(rangeImage + 199) << std::endl;
        double currentPosition[3] = { cr->getPosition()[0], cr->getPosition()[1], cr->getPosition()[2] };

        cr->refreshDisplay(rangeImage);

        bool obstacles = false;     //czy jakieœ przeszkody s¹ w zasiêgu lidara
        bool check = false;
        bool check2 = false;

        if (!map->isFirstTurn() && (abs(map->getMapClosurePosition()[0] - currentPosition[0]) > 1 || abs(map->getMapClosurePosition()[1] - currentPosition[1] > 1)))
            map->openMap();            //mo¿liwoœæ zamkniêcia pêtli mapy

        if (map->isMapOpened() && (abs(map->getMapClosurePosition()[0] - currentPosition[0]) < 0.2) && (abs(map->getMapClosurePosition()[1] - currentPosition[1]) < 0.2)) {
            map->closeMap();
            map->finishMapping();
            cr->setMode(3);
            //std::map <double, double> ::iterator it;
            //for (it = map->getMap().begin(); it != map->getMap().end(); ++it) {
                //std::cout << it->first << " => " << it->second << std::endl;
            //}
        }
        if (cr->getMode() == 1 || cr->getMode() == 2 || cr->getMode() == 11 || cr->getMode() == 12) {
            if (*(rangeImage + 199) > 0.4 && map->isMapping() && cr->getMode() != 11 && cr->getMode() != 12)
                cr->setMode(3);
            else {
                for (int i = 0; i < 200; i++) {
                    if (*(rangeImage + i) < 1) {
                        obstacles = true;
                        if (cr->getMode() != 11 && cr->getMode() != 12)
                            cr->setMode(2);
                        //if (map->isMapping()) {
                            //double* xy = cr->calculatePoint(*(rangeImage + i), i);
                            //map->insertPoint(xy[0], xy[1]);
                        //}
                    }
                    if (*(rangeImage + i) < 0.2) {
                        cr->setMode(3);
                        if (!map->isWallFound())
                            map->beginMapping();
                        map->setWallFound();
                    }
                    //std::cout << i << "->" << *(rangeImage + i) << " ";
                }
                //std::cout << std::endl;
            }
        }

        if (cr->getMode() == 2 && obstacles == false)      //je¿eli robot jedzie i nie ma przeszkód = przyœpiesz
            cr->setMode(1);

        switch (cr->getMode()) {
        case 1: cr->driveRobot(5);  //jazda prosto z pe³n¹ prêdkoœci¹
            break;
        case 2: cr->driveRobot(2);  //jazda prosto z mniejsz¹ prêdkoœci¹
            break;
        case 3: cr->stopRobot();    //zatrzymywanie robota
            break;
        case 4: if (map->isMapping() && map->isFirstTurn())     //pocz¹tek mapowania
            cr->setMode(5);
              else if (map->isMapping() && !map->isFirstTurn()) {     //polecenia przy obje¿d¿aniu terenu
            for (int i = 0; i < 200; i++) {
                if (*(rangeImage + i) < 0.2) {
                    check2 = true;
                    if (i > 90 && i < 110) {
                        check = true;
                        break;
                    }
                }
            }
            if (check)     //przy rogu / œcianie
                cr->setMode(7);
            else if (*(rangeImage + 199) > 0.6 && !check2) {    //przy wewnêtrznym rogu
                cr->setMode(11);
                distance = 0;
            }
            else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.45 && *(rangeImage + 199) < 0.6) {      //wiêkszy obrót w przypadku uskoku na œcianie
                cr->setMode(9);
            }
            else if (*(rangeImage + 199) < *(rangeImage + 196) && *(rangeImage + 199) > 0.4 && *(rangeImage + 199) <= 0.45)    //do œciany
                cr->setMode(6);
            else   //od œciany    
                cr->setMode(8);

        }
              else
            cr->setMode(3);
            startAngle = cr->getPosition()[2];
            break;
        case 5: map->finishFirstTurn();
            map->setMapClosurePosition(currentPosition[0], currentPosition[1]);     //pocz¹tek tworzenia mapy
            if (*(rangeImage + 199) < 0.2 || *(rangeImage + 199) > 0.4)
                cr->turnRobot(startAngle, pi);
            else
                cr->stopTurningRobot();
            break;
        case 6: cr->turnRobot(startAngle, -0.1);
            break;
        case 7: cr->turnRobot(startAngle, pi / 2);
            break;
        case 8: cr->turnRobot(startAngle, 0.1);    //obroty robota w ró¿nych przypadkach
            break;
        case 9: cr->turnRobot(startAngle, -0.3);
            break;
        case 10: cr->turnRobot2(startAngle, -pi / 2);
            break;

        case 12: if (*(rangeImage + 199) > 0.6)
            cr->driveRobot(0.5);
               else
            cr->stopRobot();
            break;
        case 11:

            d = cr->driveRobotByDistance(0.5, 0.3 - distance);
            distance += d;
            startAngle = cr->getPosition()[2];
            break;
        }


        cr->refreshSensorValues();
    }
    delete cr, map;
    return 0;
}
//0.01m = 1 piksel