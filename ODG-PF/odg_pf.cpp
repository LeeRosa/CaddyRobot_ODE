#include "odg_pf.h"

ODG_pf::ODG_pf()
{
    goalX = 0.0;
    goalY = 0.0;
    repX = 0.0;
    repY = 0.0;
    attX = 0.0;
    attY = 0.0;
    kAtt = 3;
    kRep = 1;
}

ODG_pf::ODG_pf(double Att, double Rep)
{
    goalX = 0.0;
    goalY = 0.0;
    repX = 0.0;
    repY = 0.0;
    attX = 0.0;
    attY = 0.0;
    kAtt = Att;
    kRep = Rep;
}

void ODG_pf::calPF(KSensorLidar2D data, double goalX1, double goalY1)
{
//    SensorData = data;
    goalX = goalX1;
    goalY = goalY1;

        cout<<"33333333333333333333333333333333"<<endl;
//    potentialSum();
        cout<<"444444444444444444444444444444"<<endl;
}
void ODG_pf::calPF(double goalX1, double goalY1)
{
//    SensorData = data;
    goalX = goalX1;
    goalY = goalY1;

        cout<<"33333333333333333333333333333333"<<endl;
//    potentialSum();
        cout<<"444444444444444444444444444444"<<endl;
}
void ODG_pf::potentialSum()
{
    attractive();
    repulsive();
    potentialSumX = attX + repX;
    potentialSumY = attY + repY;
}

void ODG_pf::attractive()
{
    // 거리센서
    for(auto& beam : SensorData)
    {
        // printf("deg: %f, dist: %f \n", beam.second->_oInfo.deg, beam.second->_dRange);
//        map[y][x].att  = POINT2D_F( -kAtt * (x - m_goal.nX), -kAtt * (y - m_goal.nY));
        attX = -kAtt*goalX;
        attY = -kAtt*goalY;
    }
}
void ODG_pf::repulsive()
{
    double rhozero = 0.0;
    for(auto& beam : SensorData)
    {
        // printf("deg: %f, dist: %f \n", beam.second->_oInfo.deg, beam.second->_dRange);
        double rho = beam.second->_dRange;
        if(rho < rhozero)
        {
            double product = kRep*(1/rho - 1/rhozero)*(1/(rho*rho*rho));
            repX += product*rho*sin(deg2rad(beam.second->_oInfo.deg - 90));
            repY += product*rho*cos(deg2rad(beam.second->_oInfo.deg - 90));
        }

    }
}
