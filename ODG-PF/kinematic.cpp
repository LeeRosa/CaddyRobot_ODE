#include "kinematic.h"
#define rad2th 57.295779513082320876798154814105

Kinematic::Kinematic()
{

}
Kinematic::init(double r_r, double r_l, double phi, double k_p)
{
    m_r_r = r_r;  //오른쪽 바퀴 반지름
    m_r_l = r_l;  //왼쪽 바퀴 반지름
    m_phi = phi;
    m_k_p = k_p;
}

Kinematic::set(double F_x, double F_y, double thetha_now)
{

    double  dUx = (F_x/(sqrt(pow(F_x, 2) + pow(F_y, 2))));
    double  dUy = (F_y/(sqrt(pow(F_x, 2) + pow(F_y, 2))));
  //  printf("dUx: %f   ",dUx);
  //  printf("dUy: %f \n",dUy);
  //    m_thetha_d = atan2(dUy, dUx);
  //    m_omega = (m_thetha_d - thetha_now)*rad2th;                    // 회전하는 속도

  //  printf("dUx: %f   ",dUx);
  //  printf("dUy: %f \n",dUy);
      m_thetha_d = atan2(dUy, dUx);
      m_omega = m_k_p*(m_thetha_d - thetha_now);                    // 회전하는 속도
      m_v = m_phi*(dUx*cos(thetha_now) + dUy*sin(thetha_now));   // 직진하는 속도

  //    m_phi_r = (m_v + m_omega)/m_r_r;
  //    m_phi_l = (m_v - m_omega)/m_r_l;

      KMatrix T(2, 2);
      KVector zeta(m_v, m_omega);
      T._ppA[0][0] = m_r_r/2.;
      T._ppA[0][1] = m_r_r/2.;
      T._ppA[1][0] = m_r_r/(double)2;
      T._ppA[1][1] = -1*m_r_r/(double)2;

      KMatrix Tinv = T.Iv();

      KVector omega = Tinv*zeta;

      m_phi_r = omega[0];
      m_phi_l = omega[1];

}
Kinematic::inverseKinematic()
{

}
