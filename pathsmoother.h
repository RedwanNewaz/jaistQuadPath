#ifndef PATHSMOOTHER_H
#define PATHSMOOTHER_H
#include "stack.h"
class pathsmoother
{
public:
    pathsmoother();
    QVector<QPoint> waypoints;
    double angleThreshod,maxCurvature;
    double totalTime,timeStep,robotTrackWidth;
    qreal slope(QPoint, QPoint);
    int DirChange(QVector<QPoint>);
    float length(QVector<QPoint>);
    QVector<QPointF> splinePath();


private:
    QVector<QPoint> knots;
    QPoint MidPoint(QPoint, QPoint);
    double avgDist,alpha,Len;


protected:
	void insertMidPoints();
    float p2pLength(QPointF, QPointF);
	float curvature(float );


};

#endif // PATHSMOOTHER_H
