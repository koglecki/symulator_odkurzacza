#include "CleaningRobot.hpp"
#include <iostream>
#include <cmath>

    CleaningRobot::CleaningRobot() {
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

    void CleaningRobot::setMode(int m) {
        mode = m;
    }

    int CleaningRobot::getMode() {
        return mode;
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

    void CleaningRobot::setDriveParameters(double leftVoltage, double rightVoltage) {
        double leftRotatingSpeed = leftVoltage * 3; // rad/s      max 5V
        double rightRotatingSpeed = rightVoltage * 3; // 1 obrót = 6.2832 rad = 2 * pi
        double vleft = leftRotatingSpeed * wheelRadius; // m/s (prêdkoœæ lewego ko³a)
        double vright = rightRotatingSpeed * wheelRadius; // m/s (prêdkoœæ prawego ko³a)
        //std::cout << "vleft = " << vleft << " m/s  , vright = " << vright << " m/s" << std::endl;

        right_motor->setVelocity(rightRotatingSpeed);
        left_motor->setVelocity(leftRotatingSpeed);
    }

    void CleaningRobot::clearRobotDisplay() {
        display->setColor(0x000000);
        display->drawOval(position[0] * 100 + displayWidth / 2, -position[1] * 100 + displayHeight / 2, 15, 15);   //usuniêcie starej pozycji robota z ekranu
        display->fillOval(position[0] * 100 + displayWidth / 2, -position[1] * 100 + displayHeight / 2, 15, 15);
    }

    const float* CleaningRobot::calculatePosition() {
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

    void CleaningRobot::refreshDisplay(const float* rangeImage) {
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

    void CleaningRobot::refreshSensorValues() {
        prevPoseSensor[0] = poseSensor[0];
        prevPoseSensor[1] = poseSensor[1];
    }

    double* CleaningRobot::calculatePoint(double distance, int i) {
        double x2 = position[0] + (robotRadius * cos(position[2]));
        double y2 = position[1] + (robotRadius * sin(position[2]));

        double x3 = x2 + (distance * cos(position[2] + pi / 2 - i * (pi / (lidar->getHorizontalResolution() - 1))));
        double y3 = y2 + (distance * sin(position[2] + pi / 2 - i * (pi / (lidar->getHorizontalResolution() - 1))));
        x3 = round(x3 * 100) / 100;
        y3 = round(y3 * 100) / 100;
        double xy[2] = { x3, y3 };
        return xy;
    }

    void CleaningRobot::stopRobot() {                      //zatrzymanie robota
        setDriveParameters(0, 0);
        if (getPoseSensor()[0] - getPrevPoseSensor()[0] == 0)
            if (mode == 11)
                mode = 10;
            else
                mode = 4;
    }

    void CleaningRobot::turnRobot(double startAngle, double angle) {      //obrót o konkretny k¹t
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

    void CleaningRobot::turnRobot2(double startAngle, double angle) {
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

    void CleaningRobot::driveRobot(double voltage) {       // jazda prosto z jedn¹ nastaw¹
        setDriveParameters(voltage, voltage);
    }

    double CleaningRobot::driveRobotByDistance(double voltage, double distance) {
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

    void CleaningRobot::stopTurningRobot() {                      //zatrzymanie obrotu robota
        setDriveParameters(0, 0);
        if (getPoseSensor()[0] - getPrevPoseSensor()[0] == 0)
            mode = 2;
    }

    CleaningRobot::~CleaningRobot() {
        delete robot, right_motor, left_motor, right_position, left_position, lidar, display, pen;
    }