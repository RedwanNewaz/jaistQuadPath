#include "search.h"
#include "Dstar.h"
#include <QDebug>
#include <QtCore>
#include <cmath>
#ifndef PATH_H
#define PATH_H

using namespace std;
class path
{
public:
    path();
//    variables
    float resolution;
    struct user{double x,y,z;}cmd,robot;
    struct property{
        double xmin,xmax;
        double ymin,ymax;
    }lim;
    QVector<QPoint> map;

//    functions
    bool input(QPoint, QPoint);
    QVector<QPoint> output();
    void updatemap(QVector<QPoint>);
    QVector<QPoint>traj();

    float pointDist(QPoint p, QPoint q);
    float isLeft( QPoint V, QPoint R, QPoint P);
private:
    QPoint R,goal;
    QVector<QPoint> search_space,foundPath;
protected:
    void searchSpace();
    int wn_PnPoly( QPoint );
};

#endif // PATH_H
