#-------------------------------------------------
#
# Project created by QtCreator 2015-06-14T15:06:34
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = pathPlanner
TEMPLATE = app


SOURCES += main.cpp\
        planner.cpp \
    qcustomplot.cpp

HEADERS  += planner.h \
    qcustomplot.h

FORMS    += planner.ui