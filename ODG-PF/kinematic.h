#ifndef KINEMATIC_H
#define KINEMATIC_H
#include <iostream>
#include <math.h>
#include "kfc.h"
class Kinematic
{
public:
    Kinematic();
    set(double F_x, double F_y, double thetha_now);
    inverseKinematic();
    init(double r_r, double r_l, double phi, double k_p);

public:
    double m_thetha_d = 0;
    double m_v = 0;
    double m_omega = 0;
    double m_r_r = 0;
    double m_r_l = 0;
    double m_phi = 0;
    double m_k_p = 0;
    double m_phi_r;
    double m_phi_l;
private:

};

#endif // KINEMATIC_H
