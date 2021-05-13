// ODE 켰을때 빨간점이 X 축, 파란점이 Y축


#include <iostream>

using namespace std;

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "sensorBeam.h"
#include <iostream>
#include <fstream>
#include "kinematic.h"
#include "odg_pf.h"
#include <random>
#include <iomanip>
#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#endif

#define NUM 4

#define SENSOR_DIST 5.0
#define OBS_NUM 8
dWorldID world;
dSpaceID space;
dGeomID  ground;
dJointGroupID contactgroup;
dsFunctions fn;


KSensorLidar2D	_oLidar2D; //BeamSensor를 저장하는 Hash Map

typedef struct {
    dBodyID body;
    dGeomID geom;
    double  l,r,m;
} MyObject;
typedef struct{
    double deg;
    double dist;
}BeamInfo_;

dReal   sim_step = 0.001;

#define KATT 0.5
#define KREP 0.5
#define GOALX 5.0
#define GOALY 3.0
Kinematic g_Kinematic;
ODG_pf g_PF(KATT, KREP);

MyObject wheel[NUM], base, neck, camera, ball, obs[OBS_NUM];
dJointID joint[NUM], neck_joint, camera_joint;
static const dReal  BALL_R    = 0.15;
static const dReal  BALL_P[3] = {1.0, 3.1, 0.15};
static const dReal  OBS_BAX[3] = {0.49,0.49,0.5}; // 장애물 크기
dReal  OBS_BAX_Px[OBS_NUM];
dReal  OBS_BAX_Py[OBS_NUM];
dReal  OBS_BAX_Pz[OBS_NUM];

dReal w_r = 0.0, w_l = 0.0 ;                   // 좌우 바퀴 속도
dReal BASE_M    = 30.0;                        // 베이스의 질량
dReal BASE_S[3] = {0.8,0.4,0.7};              // 베이스 크기

static const dReal  CAMERA_M  = 0.2;
static const dReal  CAMERA_S[3]  = {0.2,0.2,0.05};
static const dReal  NECK_M    = 0.5;
static const dReal  NECK_L    = 0.2;
static const dReal  NECK_R    = 0.025;

dReal WL;   //바퀴 축 길이
dReal WH_R0 = 0.075, WH_R1 = 0.15;            // 바퀴 반경
dReal WH_W  = 0.02, WH_M0 = 0.1, WH_M1 = 0.2; // 바퀴 폭, 질량
dReal START_X = 0, START_Y = 0, START_Z=0.60; // 초기위치

static dReal s_x = 0.0, s_y = 0.0, s_t = 0;     // deadreckoning

static double cnt2 = 0;


void calPF(double goal2X, double goal2Y);
void potentialSum();
void attractive();
void repulsive();
static double deg2rad(double x) { return x * M_PI / 180.0; }
double potentialSumX;
double potentialSumY;
KSensorLidar2D SensorData;
double goalX;
double goalY;
double repX=0;
double repY=0;
double attX=0;
double attY=0;
double kRep;
double kAtt;
vector<double> drawMin;

bool saveFlag = false;
bool startFlag = false;

static void saveBeam()
{
    ofstream writeFile("Lidar_data.txt");
    if(writeFile.is_open())
    {
        for (auto& beam : _oLidar2D)
        {
            writeFile <<beam.second->_oInfo.deg<<","<<beam.second->_dRange<<"\n";
        }
    }
    writeFile.close();

}

static void savePF()
{
    ofstream writeFile("PF_data.txt");
    if(writeFile.is_open())
    {

        writeFile <<"potentialSumX: "<<potentialSumX<<","<<"potentialSumY: "<<potentialSumY<<"\n";
    }
    writeFile.close();
}

static void makeBall()
{

    dMass m1;

    dReal  ball_mass   = 0.45;
    ball.body    = dBodyCreate(world);

    dMassSetZero(&m1);
    dMassSetSphereTotal(&m1,ball_mass,BALL_R);
    dMassAdjust(&m1,ball_mass);
    dBodySetMass(ball.body,&m1);

    ball.geom = dCreateSphere(space,BALL_R);
    dGeomSetBody(ball.geom,ball.body);
    dBodySetPosition(ball.body,BALL_P[0],BALL_P[1],BALL_P[2]);

}

static void makeObstacle()
{
    // Obstacle_position.txt
    ifstream fin("Obstacle_position.txt");

    if (!fin.is_open()) {
        cerr << "Could not open the file - '"
             << "Obstacle_position'" << endl;
        //        return EXIT_FAILURE;
    }

    int cnt = 0;
    while (!fin.eof())
    {
        fin >> OBS_BAX_Py[cnt];
        fin >>  OBS_BAX_Px[cnt];
        fin >>  OBS_BAX_Pz[cnt++];

    }
    dMass mass;
    dReal obs_mass = 5.0;
    for(int i = 0; i < OBS_NUM; i++)
    {
        obs[i].body = dBodyCreate(world);
        dMassSetZero(&mass);
        dMassSetBoxTotal(&mass, obs_mass, OBS_BAX[0], OBS_BAX[1], OBS_BAX[2]);
        dBodySetMass(obs[i].body, &mass);

        obs[i].geom = dCreateBox(space, OBS_BAX[0], OBS_BAX[1], OBS_BAX[2]);
        dGeomSetBody(obs[i].geom, obs[i].body);
        dBodySetPosition(obs[i].body, OBS_BAX_Px[i],OBS_BAX_Py[i], OBS_BAX_Pz[i]);
    }

}


static void makeRobot()
{
    dMatrix3 R;
    dMass mass, mass2, mass3;

    //베이스
    base.body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,BASE_M,BASE_S[0],BASE_S[1],BASE_S[2]);
    dBodySetMass(base.body,&mass);

    base.geom = dCreateBox(space,BASE_S[0],BASE_S[1],BASE_S[2]);
    dGeomSetBody(base.geom,base.body);
    dBodySetPosition(base.body,START_X,START_Y,START_Z);

    //목
    neck.body  = dBodyCreate(world);
    dMassSetZero(&mass2);
    dMassSetCapsuleTotal(&mass2,NECK_M,3,NECK_R,NECK_L); //인수 3은 capsule 장축이 z축
    dMassAdjust(&mass2,NECK_M);
    dBodySetMass(neck.body,&mass2);

    neck.geom = dCreateCCylinder(space,NECK_R,NECK_L);
    dGeomSetBody(neck.geom,neck.body);
    dBodySetPosition(neck.body,START_X + 0.5*BASE_S[0] - NECK_R,
            START_Y, START_Z + 0.5*BASE_S[2] + 0.5*NECK_L);
    dGeomDisable(neck.geom);

    //카메라
    camera.body  = dBodyCreate(world);
    dMassSetZero(&mass3);
    dMassSetBoxTotal(&mass3,CAMERA_M,CAMERA_S[0],CAMERA_S[1],CAMERA_S[2]);
    dMassAdjust(&mass3,CAMERA_M);
    dBodySetMass(camera.body,&mass3);

    camera.geom = dCreateBox(space,CAMERA_S[0],CAMERA_S[1],CAMERA_S[2]);
    dGeomSetBody(camera.geom,camera.body);
    dBodySetPosition(camera.body,START_X + 0.5*BASE_S[0] - NECK_R, START_Y,
            START_Z + 0.5*BASE_S[2] + NECK_L);
    dGeomDisable(camera.geom);


    //바퀴
    dRFromAxisAndAngle(R,1,0,0, M_PI/2.0);
    for (int i = 0; i < NUM; i++) { // NUM 바퀴의 개수 4개 양옆에 큰 바퀴 앞뒤로 작은 바퀴
        wheel[i].body = dBodyCreate(world);

        dMassSetZero(&mass);
        if ((i % 2) == 0) { // 앞뒤의 작은 바퀴 0번, 2번
            if(i==2)  continue;  // 뒤에 작은 바퀴는 안만들기
            dMassSetCylinderTotal(&mass,WH_M0, 2, WH_R0, WH_W);
            dBodySetMass(wheel[i].body,&mass);
            wheel[i].geom = dCreateCylinder(space, WH_R0, WH_W);
        }
        else { // 양옆의 큰 바퀴 1번, 3번
            dMassSetCylinderTotal(&mass,WH_M1, 2, WH_R1, WH_W);
            dBodySetMass(wheel[i].body,&mass);
            wheel[i].geom = dCreateCylinder(space, WH_R1, WH_W);
        }
        dGeomSetBody(wheel[i].geom, wheel[i].body);
        dBodySetRotation(wheel[i].body,R);
    }

    WL        = BASE_S[1]+WH_W;

    // 여기서 부터 바퀴의 위치 결정 w1은 왼쪽 바퀴, w0는 앞바퀴
    dReal w1y = 0.5 * (BASE_S[1] + WH_W);
    dReal w1z = START_Z - 0.5 * BASE_S[2];
    dReal w0x = 0.5 * BASE_S[0] - WH_R0;
    dReal w0z = START_Z - 0.5 * BASE_S[2] - WH_R0;

    dBodySetPosition(wheel[0].body,    w0x,  0, w0z);
    dBodySetPosition(wheel[1].body,   -w0x+WH_R0,  w1y, w1z);
    dBodySetPosition(wheel[3].body,   -w0x+WH_R0, -w1y, w1z);

    //힌지조인트
    for (int i = 0; i < NUM; i++) {
        if(i==2)  continue;  // 뒤에 작은 바퀴는 안만들기
        joint[i] = dJointCreateHinge(world,0);
        dJointAttach(joint[i], base.body, wheel[i].body); // 베이스와 바퀴랑 힌지 조인트로 연결 할꺼임
    }

    dJointSetHingeAxis(joint[0],0, -1, 0); // Y축 방향으로 회전 한다고 해서 -1 붙음
    dJointSetHingeAxis(joint[1],0, -1, 0);
    dJointSetHingeAxis(joint[3],0, -1, 0);

    dJointSetHingeAnchor(joint[0],    w0x,  0, w0z);
    dJointSetHingeAnchor(joint[1],   -w0x+WH_R0,  w1y, w1z);
    dJointSetHingeAnchor(joint[3],   -w0x+WH_R0, -w1y, w1z);

    //목 조인트
    neck_joint = dJointCreateHinge(world,0);
    dJointAttach(neck_joint,neck.body,base.body);
    dJointSetHingeAxis(neck_joint,0, 0, 1);
    dJointSetHingeAnchor(neck_joint, START_X + 0.5*BASE_S[0]-NECK_R,
            START_Y, START_Z + 0.5*BASE_S[0]);
    dJointSetHingeParam(neck_joint,dParamLoStop, 0);
    dJointSetHingeParam(neck_joint,dParamHiStop, 0);


    //카메라 조인트
    camera_joint = dJointCreateHinge(world,0);
    dJointAttach(camera_joint,neck.body,camera.body);
    dJointSetHingeAxis(camera_joint,1,0,0);
    dJointSetHingeAnchor(camera_joint,START_X + 0.5*BASE_S[0]-NECK_R,
            START_Y, START_Z + 0.5*BASE_S[0] + NECK_L);
    dJointSetHingeParam(camera_joint,dParamLoStop, 0.0);
    dJointSetHingeParam(camera_joint,dParamHiStop, 0.0);

    //Lidar 2D 센서
    KSENSORLIDAR2D_INFO	oInfo;
    OBJECT_ODE			oBase = { base.body, base.geom };	//Beam Sensor를 부착할 Object

    strcpy(oInfo.szID, "SensorLidar2D");
    oInfo.fR			= oInfo.fG = oInfo.fB = 1.0; // R G B 가 1이니까 레이의 색깔은 흰색
    oInfo.dX			= START_X + 0.5*BASE_S[0]; // 베이스의 앞부분으로 하기위해 + 0.5*베이스 / 만약 베이스 중심이면 그냥 START_X 만
    oInfo.dY          = START_Y;
    oInfo.dZ          = START_Z/2.0;
    oInfo.dMaxLength	= SENSOR_DIST;
    oInfo.nLoRange	= -90;
    oInfo.nHiRange	= 90;
    oInfo.oBase       = oBase;

    _oLidar2D.Create(oInfo, world, space);
}

static void command(int cmd)
{
    switch (cmd) {
    case 'j': w_r += 0.1; break;
    case 'f': w_l += 0.1; break;
    case 'k': w_r -= 0.1; break;
    case 'd': w_l -= 0.1; break;
    case 's': w_r = w_l = 0.0;break;
    case 'q':saveFlag = true; break;
    case 'o':startFlag = !startFlag; break;
    default: printf("\nInput j,f,k,d,l,s \n");break;
    }
}

static void control() {
    double fMax = 100;         // 최대 토크[Nm]

    dJointSetHingeParam(joint[1],  dParamVel, w_l ); // 왼쪽의 각속도
    dJointSetHingeParam(joint[1], dParamFMax, fMax); // 왼쪽의 각속도에 도달하기 위한 최대 토크
    dJointSetHingeParam(joint[3],  dParamVel, w_r ); // 오른쪽의 각속도
    dJointSetHingeParam(joint[3], dParamFMax, fMax); // 오른쪽의 각속도에 도달하기 위한 최대 토크
}

static void start() // 여기서 처음 시작 하는 viewPoint따짐 // 뷰포인트 뷰 포인트
{
    float xyz[3] = {  3.f,   0.0f, 9.f}; // (x,y,z)
    float hpr[3] = { 0.0f, -90.0f, 0.0f}; // 이거는 회전 Y축방향으로 90도 회전

    dsSetViewpoint(xyz,hpr);
    dsSetSphereQuality(2);
}

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{

    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

    static const int nMAX_CONTACT = 10;
    dContact contact[nMAX_CONTACT];

    //충돌점 계산
    int nNumContact = dCollide(o1, o2, nMAX_CONTACT, &contact[0].geom, sizeof(dContact));

    //센싱 / 충돌한게 라이다 센서인 레이라면 그냥 return으로 끝냄
    if(nNumContact == 0 || _oLidar2D.Measure(contact[0].geom) == true)
        return;

    //총돌 설정 / 여기 충돌은 로봇이 장애물과 부딪힌 경우
    for(int i=0; i<nNumContact; i++)
    {
        contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.slip1 = 0.1;
        contact[i].surface.slip2 = 0.1;
        contact[i].surface.soft_erp = 0.8;
        contact[i].surface.soft_cfm = 1e-5;

        dJointAttach(dJointCreateContact(world, contactgroup, &contact[i]), b1, b2);
    }
}

static void drawObstacle()
{
    dsSetColor(1.0,1.0,1.0);
    //    cout<<"cnt2: "<<cnt2<<endl;
    for(int i = 0; i < OBS_NUM; i++)
    {
        //        dBodySetPosition(obs[i].body, OBS_BAX_Px[i]+sin(cnt2+2*M_PI),OBS_BAX_Py[i]+sin(cnt2+2*M_PI), 0.25);
        dsDrawBox(dGeomGetPosition(obs[i].geom),
                  dGeomGetRotation(obs[i].geom),OBS_BAX);
    }
}

static void drawBall()
{
    dsSetColor(1.0,0.0,0.0);
    //  dBodySetPosition(ball.body,BALL_P[0]+4*sin(cnt2*2+4*M_PI),BALL_P[1]-4*sin(cnt2*2+2*M_PI),BALL_P[2]);
    dBodySetPosition(ball.body,BALL_P[0] + 7*cnt2,BALL_P[1]-sin(cnt2*6+M_PI),BALL_P[2]);

    dsDrawSphere(dGeomGetPosition(ball.geom),
                 dGeomGetRotation(ball.geom),BALL_R);
}

static void drawRobot()
{
    //베이스
    //  dsSetColor(0.0,0.0,0.0);
    dsSetColor(1.3,1.3,0.0);
    dsDrawBox(dGeomGetPosition(base.geom),
              dGeomGetRotation(base.geom),BASE_S);

    //목
    dsSetColor(1.3,0.0,1.3);
    dsDrawCylinder(dBodyGetPosition(neck.body),
                   dBodyGetRotation(neck.body),NECK_L,NECK_R);

    //카메라
    dsSetColor(0,1.0,0);
    dsDrawBox(dGeomGetPosition(camera.geom),
              dGeomGetRotation(camera.geom),CAMERA_S);

    //바퀴
    dsSetColor(1.1,1.1,1.1);
    for (int i=0; i< NUM; i++) {
        if ((i % 2) == 0) {
            if(i==2)  continue;
            dsDrawCylinder(dBodyGetPosition(wheel[i].body),
                           dBodyGetRotation(wheel[i].body),WH_W,WH_R0);
        }
        else {
            dsDrawCylinder(dBodyGetPosition(wheel[i].body),
                           dBodyGetRotation(wheel[i].body),WH_W,WH_R1);
        }
    }

    // 거리센서
    for(auto& beam : _oLidar2D)
    {
        beam.second->_oInfo.fR = beam.second->_oInfo.fG = beam.second->_oInfo.fB = 1.0; // R G B 가 1이니까 레이의 색깔은 흰색
        for(int i = 0; i <drawMin.size(); i++)
        {
            if(beam.second->_oInfo.deg == drawMin[i])
            {
                beam.second->_oInfo.fB = 0;
                beam.second->_oInfo.fG = 0;
                beam.second->_oInfo.fR = 1;
                break;
            }
        }
        beam.second->Draw();
    }
    drawMin.clear();
}

//월드좌표계를 로봇좌표계로 변환
static void worldToRobot(const dReal p[2], const dReal c[2], dReal theta,
dReal pr[2], dReal *r, dReal *phi)
{
    pr[0] =   (p[0]-c[0]) * cos(theta) + (p[1]-c[1]) * sin(theta);
    pr[1] = - (p[0]-c[0]) * sin(theta) + (p[1]-c[1]) * cos(theta);

    *r   = sqrt(pr[0] * pr[0] + pr[1] * pr[1]);  //물체까지의 거리
    *phi = atan2(pr[1], pr[0]);                  //물체의 각도
}

//로봇방향을 구하는 함수
dReal heading()
{
    const dReal *c,*cam;

    c   = dBodyGetPosition(base.body);     //베이스 월드좌표
    cam = dBodyGetPosition(camera.body);   //카메라 월드좌표

    return atan2(cam[1]-c[1], cam[0]-c[0]); // 로봇방향
}

//로봇 비전 함수
static int vision(dReal *r, dReal *phi, dBodyID obj_body)
{
    const dReal *p,*c,*cam;
    dReal pr[2], view_angle = M_PI/4.0;  //로봇좌표계에서의 물체위치,시야갹

    p   = dBodyGetPosition(obj_body);    //물체의 월드좌표
    c   = dBodyGetPosition(base.body);   //베이스의 월드좌표
    cam = dBodyGetPosition(camera.body); //카메라의 월드좌표

    dReal theta  = atan2(cam[1] - c[1], cam[0] - c[0]); //로봇 방향

    worldToRobot(p, c, theta, pr, r, phi);

    if ((-view_angle <= *phi) && (*phi <= view_angle)) return 1;
    else                                               return 0;
}

//바퀴 제어
static void controlWheel(dReal left, dReal right)
{
    double fMax = 100;

    dJointSetHingeParam(joint[1],  dParamVel, left);
    dJointSetHingeParam(joint[1], dParamFMax, fMax);
    dJointSetHingeParam(joint[3],  dParamVel, right);
    dJointSetHingeParam(joint[3], dParamFMax, fMax);
}

//회전
static void turn(dReal speed)
{
    controlWheel(speed, -speed);
}

//추적
static void track(dReal phi, dReal speed)
{
    dReal k = 0.5;
    if (phi > 0) controlWheel(k * speed,      speed);
    else         controlWheel(    speed,  k * speed);
}

static void deadreckoning(dReal& s_x, dReal& s_y, dReal& s_t)
{
    const dReal  *c;
    dReal        theta;  // 로봇 방향
    dReal        omega_r, omega_l;
    dReal        fx, fy, ft;
    static dReal omega_l_old = 0, omega_r_old = 0, theta_old = 0;


    //로봇 월드좌표와 방향
    c     = dBodyGetPosition(base.body);
    theta = heading();

    //각속도
    omega_r = dJointGetHingeAngleRate(joint[3]);
    omega_l = dJointGetHingeAngleRate(joint[1]);

    fx      = 0.5 * WH_R1 * ((omega_r + omega_l) * cos(theta) + (omega_r_old + omega_l_old) * cos(theta_old) );
    fy      = 0.5 * WH_R1 * ((omega_r + omega_l) * sin(theta) + (omega_r_old + omega_l_old) * sin(theta_old) );
    ft      = 0.5 * WH_R1/WL* ( (omega_r - omega_l) + (omega_r_old - omega_l_old) );

    s_x    += fx * sim_step * 0.5;
    s_y    += fy * sim_step * 0.5;
    s_t    += ft * sim_step * 0.5;

    omega_r_old = omega_r;
    omega_l_old = omega_l;
    theta_old   = theta;

    //    printf("Real x = %f  Dead Reckoning s_x = %f Error = %f\n", c[0], s_x, c[0] -s_x);
    //    printf("Real y = %f  Dead Reckoning s_y = %f Error = %f\n", c[1], s_y, c[1] -s_y);
    //    printf("Real t = %f  Dead Reckoning s_t = %f Error = %f\n", theta,s_t, theta-s_t);

}

static void simLoop(int pause)
{
    dReal        r, phi; // r:물체까지의 거리, phi: 물체의 방위각

    cnt2 = cnt2 + 0.0001;
    static int cnt4 = 0;
    static int sibal = 0;
    if(sibal++ > 500)
    {
        if(!pause)
        {


            for (auto& beam : _oLidar2D)
            {
                //          printf("deg: %f, dist: %f \n", beam.second->_oInfo.deg, beam.second->_dRange);
                beam.second->_dRange = SENSOR_DIST; // 센서 데이터 초기화
            }

            dSpaceCollide (space,0,&nearCallback); // 충돌점 계산

            control(); // 모터 돌도록 설정하는 부분 값은 키보드로 넣어줌

            //     // 여기는 물체 traking 해서 움직이는 부분
            //     if (vision(&r, &phi, ball.body))
            //     {
            //          printf("tracking phi=%.2f \n",phi * 180/M_PI);
            //          track(phi,4.0);
            //     }
            //     else
            //     {
            //          printf("lost the ball phi=%.2f \n",phi * 180/M_PI);
            //          turn(4.0);
            //     }
            //     for(auto _lidar)

            const dReal *r = dBodyGetRotation(base.body);
            double theta = atan2(-r[1],r[0]);

//            if(cnt4++%10)
//            {
            if(startFlag == true)
            {
                const dReal  *c;
                c     = dBodyGetPosition(base.body);

                double _goalX = GOALX - c[0];
                double _goalY = GOALY - c[1];
                double dist = sqrt(_goalX*_goalX + _goalY*_goalY);
                if(dist > 0.3)
                {
                    //             g_PF.calPF(_oLidar2D,_goalX,_goalY);

                    calPF(_goalX,_goalY);
                    //             g_Kinematic.set(g_PF.potentialSumX, g_PF.potentialSumY, theta);
                    g_Kinematic.set(potentialSumX, potentialSumY, theta);
                    w_r = g_Kinematic.m_phi_r;//*0.5*0.5;
                    w_l = g_Kinematic.m_phi_l;//*0.5*0.5;
                }
                else
                {
                    w_r = 0;
                    w_l = 0;
                }
                if(saveFlag == true)
                {
                    savePF();
                    saveFlag = false;
                }
                cnt4=0;
            }


//            }

            //     repulsive();


            //동력학 계산
            dWorldStep(world, sim_step);

            //접촉점 그룹 비움
            dJointGroupEmpty (contactgroup);

            deadreckoning(s_x, s_y, s_t);
        }
    }

    //  drawBall();
    drawRobot();
    drawObstacle();
}

static void setDrawStuff() {
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command; // 키보드 input
    fn.path_to_textures = "c:/ode-0.13/drawstuff/textures";
}

int main(int argc, char *argv[])
{
    dInitODE();                                   // initalize ODE
    setDrawStuff();                               // set drawstuff

    g_Kinematic.init(0.15,0.15, 0.3, 3);


    world        = dWorldCreate();                // create a world
    space        = dHashSpaceCreate(0);           //
    contactgroup = dJointGroupCreate(0);
    ground       = dCreatePlane(space,0,0,1,0);

    dWorldSetGravity(world, 0, 0, -9.8);          // set gravity

    makeObstacle();
    makeRobot();
    //  makeBall();
    dsSimulationLoop(argc,argv,640,480,&fn);

    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}



void calPF(double goal2X, double goal2Y)
{
    goalX = goal2X;
    goalY = goal2Y;
    repX = 0.0;
    repY = 0.0;
    attX = 0.0;
    attY = 0.0;
    kAtt = KATT;
    kRep = KREP;
    potentialSum();
}

void potentialSum()
{
    attractive();
    repulsive();
    //    cout<<"attX: "<<attX<<"    attY: "<<attY<<endl;
    //    cout<<"repX: "<<repX<<"    repY: "<<repY<<endl;
    potentialSumX = attX + repX;
    potentialSumY = attY + repY;
    cout<<"potentialSumX: "<<potentialSumX<<"    potentialSumY: "<<potentialSumY<<endl;
}

void attractive()
{
    // 거리센서
    //    for(auto& beam : _oLidar2D)
    //    {
    //        // printf("deg: %f, dist: %f \n", beam.second->_oInfo.deg, beam.second->_dRange);
    //        map[y][x].att  = POINT2D_F( -kAtt * (x - m_goal.nX), -kAtt * (y - m_goal.nY));
    attX = -kAtt*(-goalX);
    attY = -kAtt*(-goalY);
    //    }
}



void repulsive()
{
    double rhozero = 1.0;
    //    cout<<"rhozero: "<<rhozero<<endl;
    vector<double> dist(181);

    std::srand(std::time(nullptr));
    // deg: -90 ~ 90
    for(auto& beam : _oLidar2D)
    {
//        double noise = (float)(rand())/((float)(RAND_MAX/(1)));
//        if(noise < 0.1) // max
//        {
//            dist[beam.second->_oInfo.deg + 90] = SENSOR_DIST;
//        }
//        else if (noise < 0.2) // random
//        {
//            double random_noise = (float)(rand()) / (float)(SENSOR_DIST);
//            dist[beam.second->_oInfo.deg + 90] = random_noise;
//        }
//        else // gaussian
//        {
//            dist[beam.second->_oInfo.deg + 90] =  (double)beam.second->_dRange/* + gaussian */;
//        }
        dist[beam.second->_oInfo.deg + 90] =  (double)beam.second->_dRange;
    }

    vector<BeamInfo_> beamdata;
    bool reset = false;
    int cnt44 = 0;
    for(int i = 0; i < dist.size()-5; i++)
    {
        double minValue = 99.0;
        double minIdx=0.0;
        double kernal[5] = {dist[i+0],
                            dist[i+1],
                            dist[i+2],
                            dist[i+3],
                            dist[i+4]};

        for(int k = 0; k < 5; k++) // kernel 만들고 최소 값 구하기
        {
            //            cout<<"kernel["<<k<<"]: "<<kernal[k]<<endl;
            if(dist[i+k] > rhozero)
            {
                if(reset == false)
                {
                    BeamInfo_ newData;
                    newData.deg = 0.0;
                    newData.dist = 10.0;
                    beamdata.push_back(newData);
                }
                reset = true;
                break;
            }
            else
                reset = false;
            if(kernal[k] < minValue)
            {
                minValue = kernal[k];
                minIdx = i + k -90.0;
                //                cout<<"i: "<<i<<"   k: "<<k<<"   minValue: "<<minValue<<endl;
            }
        }
        if(reset == false)
        {
            cnt44++;
            if(beamdata.size() == 0)
            {
                BeamInfo_ newData;
                newData.deg = 0;
                newData.dist = 10;
                beamdata.push_back(newData);
            }
            if(minValue < beamdata[beamdata.size()-1].dist)
            {

                beamdata[beamdata.size()-1].deg = minIdx;
                beamdata[beamdata.size()-1].dist = minValue;
            }
        }
    }



    cout<<"-----------------------------------------------------------------------------------"<<endl;
    for(auto& beam : _oLidar2D)
    {
        for(int i = 0; i < beamdata.size(); i++)
        {
            if( beam.second->_oInfo.deg == beamdata[i].deg)
            {
                if(beam.second->_dRange > 4) continue;
                cout<<"org_deg: "<<beam.second->_oInfo.deg<<"  org_dist: "<<beam.second->_dRange<<endl;
                cout<<"deg: "<<beamdata[i].deg<<"  dist: "<<beamdata[i].dist<<endl;
            }
        }
    }
    cout<<"beamdata.size(): "<<beamdata.size()<<endl;
    cout<<"-----------------------------------------------------------------------------------"<<endl;


    for(int i = 0; i < beamdata.size(); i++)
    {
        double rho = beamdata[i].dist;
        if(rho < rhozero)
        {
            rho -=0.1;
            drawMin.push_back(beamdata[i].deg);

            double product = kRep*(1/rho - 1/rhozero)*(1/(rho*rho*rho));
            cout<<"product: "<<product<<endl;
            repX += product*rho*sin(deg2rad(beamdata[i].deg));
            repY += product*rho*cos(deg2rad(beamdata[i].deg));
        }
    }


    //    for(auto& beam : _oLidar2D)
    //    {
    //        // printf("deg: %f, dist: %f \n", beam.second->_oInfo.deg, beam.second->_dRange);
    //        double rho = beam.second->_dRange;
    //        if(rho < rhozero)
    //        {
    //            double product = kRep*(1/rho - 1/rhozero)*(1/(rho*rho*rho));
    //            cout<<"product: "<<product<<endl;
    ////            repX += product*rho*sin(deg2rad(beam.second->_oInfo.deg - 90));
    ////            repY += product*rho*cos(deg2rad(beam.second->_oInfo.deg - 90));
    //            repX += product*rho*sin(deg2rad(beam.second->_oInfo.deg));
    //            repY += product*rho*cos(deg2rad(beam.second->_oInfo.deg));
    //        }
    //    }

}

//void minDist()
//{

//}
