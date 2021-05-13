#ifndef ODG_PF_H
#define ODG_PF_H

#include "sensorBeam.h"
#include <iostream>
#include <vector>

using namespace std;

class ODG_pf
{
public:
    ODG_pf();
    ODG_pf(double Att, double Rep);

    void calPF(KSensorLidar2D data, double goalX1, double goalY1);
    void calPF(double goalX1, double goalY1);
    void potentialSum();
    void attractive();
    void repulsive();
    static double deg2rad(double x) { return x * M_PI / 180.0; }
    double potentialSumX=0.0;
    double potentialSumY=0.0;

    KSensorLidar2D SensorData;
    double goalX;
    double goalY;
    double kRep;
    double kAtt;

    double repX = 0.0;
    double repY = 0.0;
    double attX = 0.0;
    double attY = 0.0;


private:

};

#endif // ODG_PF_H
