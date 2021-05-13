#include "odethread.h"

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

dWorldID world;
dSpaceID space;
dGeomID  ground;
dJointGroupID contactgroup;
dsFunctions fn;

typedef struct {
    dBodyID body;
    dGeomID geom;
    double  l,r,m;
} MyObject;

dReal   sim_step = 0.002;// 0.001

MyObject wheel[NUM], base;
dJointID joint[NUM];

dReal v_r = 0.0,v_l = 0.0 ;                   // 좌우 바퀴 속도
dReal BASE_M    = 10;                        // 베이스의 질량
dReal BASE_S[3] = {0.25,0.25,0.2};              // 베이스 크기
dReal START_X = 5, START_Y = 5, START_Z=0.20; // 초기위치
dReal WH_R0 = 0.05, WH_R1 = 0.101;            // 바퀴 반경
dReal WH_W  = 0.02, WH_M0 = 0.1, WH_M1 = 0.2; // 바퀴 폭, 질량

// ----------- ---- -----------

ObstacleDetect Obstacle;
PotentialField *Potential;
Kinematic g_Kinematic;
MyObject ObstacleMap[1000];
MyObject Goal;
dReal OBSATACLEBOX_S[3] = {0.49,0.49,0.5};

int mapObstacleBox = 0;
int mapObstacleCylinder = 0;
dReal CY_R0 = 0.75, CY_W  = 0.5, CY_M0 = 0.9; //

dReal MAPBOX_M = 1000;

//바퀴 제어
static void controlWheel(dReal left, dReal right)
{
    v_l = left;
    v_r = right;
  double fMax = 500;

  dJointSetHingeParam(joint[1],  dParamVel, left);
  dJointSetHingeParam(joint[1], dParamFMax, fMax);
  dJointSetHingeParam(joint[3],  dParamVel, right);
  dJointSetHingeParam(joint[3], dParamFMax, fMax);

}
//추적
static void track(dReal phi, dReal speed)
{
  dReal k = 0.5;
  if (phi > 0) controlWheel(k * speed * 2 ,  speed * 2);
  else         controlWheel(speed * 2,  k * speed * 2);
}

static void makeMap()
{

    dMass mass;
    Goal.body  = dBodyCreate(world);
    dMassSetZero(&mass);

    dMassSetBoxTotal(&mass,MAPBOX_M,OBSATACLEBOX_S[0],OBSATACLEBOX_S[1],OBSATACLEBOX_S[2]);
    dBodySetMass(Goal.body,&mass);
    Goal.geom = dCreateBox(space,OBSATACLEBOX_S[0],OBSATACLEBOX_S[1],OBSATACLEBOX_S[2]);
    dGeomSetBody(Goal.geom,Goal.body);
    dBodySetPosition(Goal.body, Potential->m_goal.nX*0.5, Potential->m_goal.nY*0.5, 0.5);

    int mapSize = 0;

    for(int i = 0; i < Obstacle.mObs.size(); i++)
    {
        if(Obstacle.map[Obstacle.mObs[i].nY][Obstacle.mObs[i].nX].isObstacleCylinder == true)
            continue;
        else
        {
            dMass mass;
            ObstacleMap[mapSize].body  = dBodyCreate(world);
            dMassSetZero(&mass);

            dMassSetBoxTotal(&mass,MAPBOX_M,OBSATACLEBOX_S[0],OBSATACLEBOX_S[1],OBSATACLEBOX_S[2]);
            dBodySetMass(ObstacleMap[mapSize].body,&mass);
            ObstacleMap[mapSize].geom = dCreateBox(space,OBSATACLEBOX_S[0],OBSATACLEBOX_S[1],OBSATACLEBOX_S[2]);
            dGeomSetBody(ObstacleMap[mapSize].geom,ObstacleMap[mapSize].body);
            dBodySetPosition(ObstacleMap[mapSize].body, Obstacle.mObs[i].nX*0.5, Obstacle.mObs[i].nY*0.5, 0.5);

            mapSize++;
        }

    }
    mapObstacleBox = mapSize;
    //    printf("Obstacle.mObs.size(): %d\n",Obstacle.mObs.size());

    mapObstacleCylinder = mapObstacleBox;
    for(int i = 0; i < 2 ; i++)
    {
        if(i == 0)
        {
            int x = 4;
            int y = 5;
            dMass mass;
            ObstacleMap[mapObstacleCylinder].body = dBodyCreate(world);
            dMassSetZero(&mass);

            dMassSetCylinderTotal(&mass,MAPBOX_M, 3, CY_R0, CY_W);
            dBodySetMass(ObstacleMap[mapObstacleCylinder].body, &mass);
            ObstacleMap[mapObstacleCylinder].geom = dCreateCylinder(space, CY_R0, CY_W);
            dGeomSetBody(ObstacleMap[mapObstacleCylinder].geom, ObstacleMap[mapObstacleCylinder].body);
            dBodySetPosition(ObstacleMap[mapObstacleCylinder].body, 0.5*x, 0.5*y, 0.5);
            mapObstacleCylinder++;
        }
        else
        {
            int x = 13;
            int y = 4;
            dMass mass;
            ObstacleMap[mapObstacleCylinder].body = dBodyCreate(world);
            dMassSetZero(&mass);

            dMassSetCylinderTotal(&mass,5.0, 3, CY_R0, CY_W);
            dBodySetMass(ObstacleMap[mapObstacleCylinder].body, &mass);
            ObstacleMap[mapObstacleCylinder].geom = dCreateCylinder(space, CY_R0, CY_W);
            dGeomSetBody(ObstacleMap[mapObstacleCylinder].geom, ObstacleMap[mapObstacleCylinder].body);
            dBodySetPosition(ObstacleMap[mapObstacleCylinder].body, 0.5*x, 0.5*y, 0.5);

            mapObstacleCylinder++;
        }
    }


}

static void makeRobot()
{
    dMatrix3 R;
    dMass mass;

    //베이스
    base.body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,BASE_M,BASE_S[0],BASE_S[1],BASE_S[2]);
    dBodySetMass(base.body,&mass);

    base.geom = dCreateBox(space,BASE_S[0],BASE_S[1],BASE_S[2]);
    dGeomSetBody(base.geom,base.body);
    dBodySetPosition(base.body,START_X,START_Y,START_Z);


    //바퀴
    dRFromAxisAndAngle(R,1,0,0, M_PI/2.0);
    for (int i = 0; i < NUM; i++) {
        wheel[i].body = dBodyCreate(world);

        dMassSetZero(&mass);
        if ((i % 2) == 0) {
            dMassSetCylinderTotal(&mass,WH_M0, 3, WH_R0, WH_W);
            dBodySetMass(wheel[i].body,&mass);
            wheel[i].geom = dCreateCylinder(space, WH_R0, WH_W);
        }
        else {
            dMassSetCylinderTotal(&mass,WH_M1, 3, WH_R1, WH_W);
            dBodySetMass(wheel[i].body,&mass);
            wheel[i].geom = dCreateCylinder(space, WH_R1, WH_W);
        }
        dGeomSetBody(wheel[i].geom, wheel[i].body);
        dBodySetRotation(wheel[i].body,R);
    }

    dReal w1y = 0.5 * (BASE_S[1]+WH_W);
    dReal w1z = START_Z - 0.5 * BASE_S[2];
    dReal w0x = 0.5 * BASE_S[0] - WH_R0;
    dReal w0z = START_Z - 0.5 * BASE_S[2] - WH_R0;

    dBodySetPosition(wheel[0].body,    w0x+START_X,  START_Y, w0z); //앞 바퀴
    dBodySetPosition(wheel[1].body,    START_X,  w1y+START_Y, w1z); //왼 바퀴
    dBodySetPosition(wheel[2].body,   -w0x+START_X,  START_Y, w0z); //뒷 바퀴
    dBodySetPosition(wheel[3].body,    START_X, -w1y+START_Y, w1z); //오른 바퀴

    //힌지조인트
    for (int i = 0; i < NUM; i++) {
        joint[i] = dJointCreateHinge(world,0);
        dJointAttach(joint[i], base.body, wheel[i].body);
    }

    dJointSetHingeAxis(joint[0],0, -1, 0);      //앞 바퀴
    dJointSetHingeAxis(joint[1],0, -1, 0);      //왼 바퀴
    dJointSetHingeAxis(joint[2],0, -1, 0);      //뒷 바퀴
    dJointSetHingeAxis(joint[3],0, -1, 0);      //오른 바퀴
    dJointSetHingeAnchor(joint[0],    w0x+START_X,  START_Y, w0z);
    dJointSetHingeAnchor(joint[1],    START_X,  w1y+START_Y, w1z);  //왼 바퀴
    dJointSetHingeAnchor(joint[2],   -w0x+START_X,  START_Y, w0z);
    dJointSetHingeAnchor(joint[3],    START_X, -w1y+START_Y, w1z);  //오른 바퀴

}

static void command(int cmd)
{
    switch(cmd)
    {
    case 'd': g_Kinematic.m_omega += 1; break;
    case 'r': g_Kinematic.m_v += 1; break;
    case 'g':  g_Kinematic.m_omega -= 1; break;
    case 'f': g_Kinematic.m_v -= 1 ; break;
    case 's': g_Kinematic.m_v = g_Kinematic.m_omega = 0.0;break;
    }
}

static void control()
{
    double fMax = 300;         // 최대 토크[Nm]

    dJointSetHingeParam(joint[1], dParamVel, v_l );
    dJointSetHingeParam(joint[1], dParamFMax, fMax);
    dJointSetHingeParam(joint[3], dParamVel, v_r );
    dJointSetHingeParam(joint[3], dParamFMax, fMax);
}

static void start()
{
    float xyz[3] = { 5.0f, 5.0f, 10.0f };
    float hpr[3] = { 90.0f, -90.0f, 0.0f };

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

    //센싱
    if(nNumContact == 0)
        return;

    //총돌 설정
    for(int i=0; i<nNumContact; i++)
    {
        contact[i].surface.mode     = dContactSlip1 | dContactSlip2 |
                dContactSoftERP | dContactSoftCFM |
                dContactApprox1;
        contact[i].surface.mu       = dInfinity;
        contact[i].surface.slip1    = 0.1;
        contact[i].surface.slip2    = 0.1;
        contact[i].surface.soft_erp = 0.8;
        contact[i].surface.soft_cfm = 1e-5;

        dJointAttach(dJointCreateContact(world, contactgroup, &contact[i]), b1, b2);
    }
}

static void drawRobot()
{
    //베이스
    dsSetColor(1.3,1.3,0.0);
    dsDrawBox(dGeomGetPosition(base.geom),
              dGeomGetRotation(base.geom),BASE_S);

    //바퀴
    dsSetColor(1.1,1.1,1.1);
    for (int i=0; i< NUM; i++) {
        if ((i % 2) == 0) {
            dsDrawCylinder(dBodyGetPosition(wheel[i].body),
                           dBodyGetRotation(wheel[i].body),WH_W,WH_R0);
        }
        else {
            dsDrawCylinder(dBodyGetPosition(wheel[i].body),
                           dBodyGetRotation(wheel[i].body),WH_W,WH_R1);
        }
    }
}
static void drawMap()
{
    for (int i = 0; i < mapObstacleBox; i++)
    {
        dsSetColor(1.0, 1.0, 1.0);
        dsDrawBox(dGeomGetPosition(ObstacleMap[i].geom),
                  dGeomGetRotation(ObstacleMap[i].geom), OBSATACLEBOX_S);
    }


    for(int i = mapObstacleBox ; i < mapObstacleCylinder ; i++)
    {
        dsSetColor(1.0, 1.0, 1.0);
        dsDrawCylinder(dBodyGetPosition(ObstacleMap[i].body),
                       dBodyGetRotation(ObstacleMap[i].body), CY_W ,CY_R0);
    }

    dsSetColor(1.5, 0, 0);
    dsDrawBox(dGeomGetPosition(Goal.geom),
              dGeomGetRotation(Goal.geom), OBSATACLEBOX_S);
}

static void simLoop(int pause)
{
    //제어
    control();

    //충돌점 계산
    dSpaceCollide (space,0,&nearCallback);

    //동력학 계산
    dWorldStep(world, sim_step);

    //접촉점 그룹 비움
    dJointGroupEmpty (contactgroup);

    //그리기
    drawRobot();
    drawMap();

    const dReal *k = dBodyGetPosition(base.body);
    float fX = k[0];
    float fY = k[1];
    const dReal *r = dBodyGetRotation(base.body);
    double theta = atan2(-r[1],r[0]);

//    dReal  *rot = dBodyGetRotation(base.body);
//    double theta = atan2(-rot[1],rot[0]);

    int x = fX*2;
    int y = fY*2;
    double dist = sqrtf(powf(x-Potential->m_goal.nX,2)+ powf(y-Potential->m_goal.nY,2));

    if(dist>0.5)
    {
        printf("dist: %f",dist);
        printf("theta: %f\n", theta);
        printf("X: %d\n", x);
        printf("Y: %d\n", y);

        g_Kinematic.set(Potential->map[y][x].potentialSum.nX, Potential->map[y][x].potentialSum.nY,theta);

        v_r = g_Kinematic.m_phi_r;//*0.5*0.5;
        v_l = g_Kinematic.m_phi_l;//*0.5*0.5;
        printf("v_r: %f     ", v_r);
        printf("v_l: %f\n", v_l);
            printf("Potential->map[y][x].potentialSum.nX: %d            Potential->map[y][x].potentialSum.nY: %d\n",   Potential->map[y][x].potentialSum.nX,    Potential->map[y][x].potentialSum.nY);
    }
    else
    {
        v_l = 0;
        v_r =0;
    }
}

static void setDrawStuff() {
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    fn.path_to_textures = "c:/ode-0.13/drawstuff/textures";
}

ODEThread::ODEThread(int argc, char argv)
{
    Obstacle.Excute("MapData.txt");
    Potential = new PotentialField(Obstacle, POINT2D(START_X/0.5,START_Y/0.5), POINT2D(17, 3), 5.0, 20.0, 5);
//    Potential = new PotentialField(Obstacle, POINT2D(START_X/0.5,START_Y/0.5), POINT2D(2, 2), 5.0, 40.0, 5);

    Potential->potentialSum();
    g_Kinematic.init(0.5,0.5, 5, 3);
    dInitODE();
    setDrawStuff();

    world        = dWorldCreate();
    space        = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    ground       = dCreatePlane(space,0,0,1,0);

    dWorldSetGravity(world, 0, 0, -9.8);

    makeMap();
    makeRobot();
    dsSimulationLoop(argc,*argv[],640,480,&fn);
//    auto thread = QThread::create( dsSimulationLoop(argc,argv,640,480,&fn));
//    thread->start();
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
}
//ODEThread::~ODEThread()
//{
////    wait();
//}

//void ODEThread::run()
//{

//}
