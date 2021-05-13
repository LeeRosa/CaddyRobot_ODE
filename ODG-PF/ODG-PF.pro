#-------------------------------------------------
#
# Project created by QtCreator 2021-03-31T16:46:05
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11


TARGET = ODG-PF
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    sensorBeam.cpp \
    odg_pf.cpp \
    kfc.cpp \
    kinematic.cpp

# For ODE Library
DEFINES += dDOUBLE
INCLUDEPATH += C:\\ode-0.13\\include
INCLUDEPATH += C:\\ode-0.13\\ drawstuff
INCLUDEPATH += C:\\ode-0.13\\ode\\src

# For ODE Library
LIBS += C:\ode-0.13\lib\ReleaseDoubleLib\libode_double.a
LIBS += C:\ode-0.13\lib\ReleaseDoubleLib\libdrawstuff.a

# For OpenGL Library
LIBS += -lopengl32\
-lglu32\
-lgdi32\
-lglut32\
-luser32\
-lwinmm\

# For Windows Resource File
win32{ RC_FILE = C:\\ode-0.13\\drawstuff\\src\\resources.rc }

HEADERS  += mainwindow.h \
    sensorBeam.h \
    odg_pf.h \
    kfc.h \
    kinematic.h

FORMS    += mainwindow.ui
