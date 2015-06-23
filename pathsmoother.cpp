#include "pathsmoother.h"
#include "stack.h"


pathsmoother::pathsmoother()
{
    alpha=120*0.0174532925; //0<->180 larger value ==low slope
}

qreal pathsmoother::slope(QPoint P, QPoint Q){
    QLineF line(P,Q);
    return line.angle();;
}

int pathsmoother::DirChange(QVector<QPoint> map){

    qreal refSlope=0;
    waypoints.clear();
    avgDist=0;
    int count=0;
    for(int i=0;i<map.size()-1;i++){
        qreal currentSlope=slope(QPoint(map.at(i)),QPoint(map.at(i+1)));
        avgDist+=p2pLength(QPoint(map.at(i)),QPoint(map.at(i+1)))/i;

        if(abs(currentSlope-refSlope)>5){
            count++;
             waypoints.push_back(map.at(i));
        }
        refSlope=currentSlope;
    }
    waypoints.push_back(map.last());
    insertMidPoints();
//    splinePath();
    return count;
}

float pathsmoother::curvature(float u){
    double num=2*(u*sin(alpha))*(1-u);
    double den=3*Len*(2*u*u*(1-cos(alpha))*pow((u*u-2*u+1)+pow((2*u-1), 2),  3/2));
    return num/den;
}

void pathsmoother::insertMidPoints(){
    knots.clear();
    knots.push_back(waypoints.first());
    for(int i=0; i<waypoints.size()-1;i++){
//        qDebug()<<"start "<<waypoints.at(i)<<"\tmid\t"<<MidPoint(waypoints.at(i),waypoints.at(i+1))<<"\tend\t"<<waypoints.at(i+1);

        QPoint mid(MidPoint(waypoints.at(i),waypoints.at(i+1)));

        if(!knots.contains(mid))
            knots.push_back(mid);


    }
    knots.push_back(waypoints.last());
    qDebug()<<knots;
}

QPoint pathsmoother::MidPoint(QPoint P, QPoint Q){

    QLineF line(P,Q);
    float dx=line.dx()/2,dy=line.dy()/2;
    return QPoint(line.pointAt(0).x()+dx,line.pointAt(0).y()+dy);

}

float pathsmoother::length(QVector<QPoint> path){

    float pathLength=0;
    for (int i=0;i<path.size()-1;i++){
        pathLength+=p2pLength(path.at(i),path.at(i+1));
    }
    return pathLength;
}

float pathsmoother::p2pLength(QPointF P, QPointF Q){
    QLineF line(P,Q);
    return line.length();

}

bool pathsmoother::findStarting(QPoint P, QPoint Q){
    if (P.x()>Q.x()){
        line.max=P.x();
        line.min=Q.x();
        return false;
    }
    else{
        line.min=P.x();
        line.max=Q.x();
        return true;
    }
}

void pathsmoother::reuseResource(QVector<QPoint>knots){

    X.clear(); Y.clear();
    order=findStarting(knots.first(),knots.last());

    if(order){
          double max=0;
        foreach(QPoint p,knots)
        {
            if(p.x()>max)max=p.x();
            if(X.contains(p.x()))
               X.push_back(max+0.5);
            else
               X.push_back(p.x());
            Y.push_back(p.y());
        }
    }
    else{
        double min=1000;
        foreach(QPoint p,knots)
        {
            if(p.x()<min)min=p.x();
            if(X.contains(p.x()))
               X.push_front(min-0.5);
            else
               X.push_front(p.x());
            Y.push_front(p.y());
        }
    }


}




QVector<QPointF> pathsmoother::splinePath(){
    QVector<QPointF>spath;
    tk::spline s;

    //change knots X&Y change node update policy
    if(abs(knots.last().x())>=abs(knots.last().y())){
        reuseResource(knots);
        s.set_points(X.toStdVector(),Y.toStdVector());

        for(float i=line.min; i<=line.max; i++){
            QPointF node(i,s(i));
            spath.push_back(node);
        }
    }


//    qDebug()<<spath;
    if(order){
    spath.push_front(knots.first());
    spath.push_back(knots.last());
    }
    else{
        spath.push_back(knots.first());
        spath.push_front(knots.last());
    }

    qDebug()<<"order"<<order;


    return spath;
}
