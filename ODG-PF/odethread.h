#ifndef ODETHREAD_H
#define ODETHREAD_H

#include <QThread>
#include <QStringListModel>


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
//#include "obstacledetect.h"
//#include "potentialfield.h"
//#include "kinematic.h"


class ODEThread/*: public QThread*/
{
    Q_OBJECT
public:
    ODEThread(int argc, char argv);
     ~ODEThread();
//    void run();

private:
    int init_argc;
        char** init_argv;
};

#endif // ODETHREAD_H
