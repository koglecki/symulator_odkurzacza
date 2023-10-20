#include "CleaningRobot.hpp"
#include <iostream>
#include <cmath>

    CleaningRobot::CleaningRobot() {
        setRobot();
    }

    CleaningRobot::CleaningRobot(double x, double y, double theta) {
        setRobot();
        position[0] = x;
        position[1] = y;
        position[2] = theta;
    }

    void CleaningRobot::setRobot() {
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

    double* CleaningRobot::getPosition() {
        return position;
    }

    double* CleaningRobot::getPrevPoseSensor() {
        return prevPoseSensor;
    }

    double* CleaningRobot::getPoseSensor() {
        return poseSensor;
    }

    int CleaningRobot::getTimeStep() {
        return timeStep;
    }

    const float* CleaningRobot::getLidarScan() {
        const float* rangeImage = lidar->getRangeImage();
        return rangeImage;
    }

    void CleaningRobot::setDriveParameters(double leftVoltage, double rightVoltage) {
        double leftRotatingSpeed = leftVoltage * 3; // rad/s      max 5V
        double rightRotatingSpeed = rightVoltage * 3; // 1 obrót = 6.2832 rad = 2 * pi

        left_motor->setVelocity(leftRotatingSpeed);
        right_motor->setVelocity(rightRotatingSpeed);       
    }

    void CleaningRobot::clearRobotDisplay() {
        display->setColor(0x000000);
        display->drawOval(position[0] * 100 + displayWidth / 2, -position[1] * 100 + displayHeight / 2, 15, 15);   // usuniêcie starej pozycji robota z ekranu
        display->fillOval(position[0] * 100 + displayWidth / 2, -position[1] * 100 + displayHeight / 2, 15, 15);
    }

    void CleaningRobot::calculatePosition() {
        poseSensor[0] = left_position->getValue();
        poseSensor[1] = right_position->getValue();        // pobranie wartoœci z czujnika pozycji

        double sright = (poseSensor[1] - prevPoseSensor[1]) * wheelRadius;    // droga prawego ko³a
        double sleft = (poseSensor[0] - prevPoseSensor[0]) * wheelRadius;     // droga lewego ko³a

        double deltatheta = (sright - sleft) / wheelbase;       // zmiana k¹ta
        deltatheta -= 0.00945 * deltatheta;
        double s = (sleft + sright) / 2;        // droga ogólna robota

        double trasform_matrix[3] = { s * cos(position[2]), s * sin(position[2]), deltatheta };      // macierz przekszta³cenia
        double new_position[3] = { position[0] + trasform_matrix[0], position[1] + trasform_matrix[1], position[2] + trasform_matrix[2] };       // nowa pozycja

        clearRobotDisplay();
        position[0] = new_position[0];
        position[1] = new_position[1];
        position[2] = new_position[2];

        std::cout << "Pozycja:   x = " << position[0] << " , y = " << position[1] << " , theta = " << position[2] << std::endl;
    }

    void CleaningRobot::refreshDisplay(const float* rangeImage) {
        display->setColor(0xff0000);
        display->drawOval(position[0] * 100 + displayWidth / 2, -position[1] * 100 + displayHeight / 2, 15, 15);   // nowa pozycja robota na ekranie
        display->fillOval(position[0] * 100 + displayWidth / 2, -position[1] * 100 + displayHeight / 2, 15, 15);

        for (int i = 0; i < lidar->getHorizontalResolution(); i++) {     // wizualizacja odczytów lidara
            if (*(rangeImage + i) < 1)
                display->setColor(0xFF00FF);
            else
                display->setColor(0x000000);
            double L = *(rangeImage + i) * 100;     // odleg³oœæ od sensora
            double x = position[0] * 100 + displayWidth / 2 + (17 * cos(position[2]));      // odleg³oœæ od œrodka robota do lidara
            double y = -position[1] * 100 + displayHeight / 2 - (17 * sin(position[2]));

            //wspó³rzêdne punktu na ekranie
            double xx = x + (L * cos(position[2] + pi / 2 - i * (pi / (lidar->getHorizontalResolution() - 1)))); // 3.14/199 = 0.015786
            double yy = y - (L * sin(position[2] + pi / 2 - i * (pi / (lidar->getHorizontalResolution() - 1))));  // -1 bo jest 199 odcinków miêdzy punktami

            display->drawPixel(round(xx), round(yy));
        }     
    }

    void CleaningRobot::refreshSensorValues() {
        prevPoseSensor[0] = poseSensor[0];
        prevPoseSensor[1] = poseSensor[1];
    }

    double* CleaningRobot::calculatePoint(double distance, int i) {     // obliczanie wspó³rzêdnych punktu odczytanego lidarem
        double x = position[0] + (robotRadius * cos(position[2]));
        double y = position[1] + (robotRadius * sin(position[2]));     // odleg³oœæ od œrodka robota do lidara

        double xx = x + (distance * cos(position[2] + pi / 2 - i * (pi / (lidar->getHorizontalResolution() - 1))));
        double yy = y + (distance * sin(position[2] + pi / 2 - i * (pi / (lidar->getHorizontalResolution() - 1))));
        xx = round(xx * 100) / 100;
        yy = round(yy * 100) / 100;
        double xy[2] = { xx, yy };
        return xy;
    }

    bool CleaningRobot::stopRobot() {                      //zatrzymanie robota
        setDriveParameters(0, 0);
        if (poseSensor[0] - prevPoseSensor[0] == 0)
            return true;
        else
            return false;
    }

    bool CleaningRobot::turnRobot(double startAngle, double angle) {      //obrót o konkretny k¹t
        bool isFinished = false;
        if ((angle >= 0 && position[2] > startAngle + angle) || (angle < 0 && position[2] < startAngle + angle)) {
            stopRobot();                                    //zatrzymywanie robota po obrocie
            if (poseSensor[0] - prevPoseSensor[0] == 0) {
                if (position[2] >= pi * 2)
                    position[2] = position[2] - pi * 2;
                else if (position[2] < 0)
                    position[2] = position[2] + pi * 2;
                return true;
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
        return false;
    }

    void CleaningRobot::driveRobot(double voltage) {       // jazda prosto z jedn¹ nastaw¹
        setDriveParameters(voltage, voltage);
    }

    double CleaningRobot::calculateDistance() {         // droga przebyta w jednostce czasu
        double s = 0;
        double sright = (poseSensor[1] - prevPoseSensor[1]) * wheelRadius;
        double sleft = (poseSensor[0] - prevPoseSensor[0]) * wheelRadius;
        s = (sleft + sright) / 2;
        return s;
    }

    void CleaningRobot::clearDisplay() {
        display->setColor(0x000000);
        display->drawRectangle(0, 0, displayWidth, displayHeight);
        display->fillRectangle(0, 0, displayWidth, displayHeight);
    }

    void CleaningRobot::drawMap(std::vector <std::vector<bool>> map, std::vector <std::vector<bool>> grid) {      // rysowanie zapisanej mapy
        display->setColor(0xffff00);
   
        for (int i = 0; i < map.size(); i++) {
            for (int j = 0; j < map[i].size(); j++) {
                if (map[i][j])
                    display->drawPixel(j + 50, i + 51);
            }           
        }

        display->setColor(0x00ffff);
        for (int i = 10; i < map.size(); i += 35) {
            for (int j = 10; j < map[i].size(); j += 35) {
                 display->drawRectangle(j + 50, i + 51, 35, 35);
            }
        }
        std::cout << "gridY = " << grid.size() << ", gridX = " << grid[0].size() << std::endl;
        for (int g = 0; g < grid.size(); g++) {
            for (int h = 0; h < grid[g].size(); h++) {
                if (grid[g][h])
                    std::cout << "1 ";
                else
                    std::cout << "0 ";
            }
            std::cout << std::endl;
        }

        /*std::cout << std::endl << "map" << std::endl;
        for (int g = 45; g < 80; g++) {
            for (int h = 335; h < 370; h++) {
                if (map[g][h])
                    std::cout << "1 ";
                else
                    std::cout << "0 ";
            }
            std::cout << std::endl;
        }*/
    }

    CleaningRobot::~CleaningRobot() {
        delete robot, right_motor, left_motor, right_position, left_position, lidar, display, pen;
    }