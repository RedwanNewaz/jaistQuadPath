#include "path.h"
#include <QtAlgorithms>

path::path()
{
    lim.xmax=0;lim.xmin=0;
    lim.ymax=0;lim.ymin=0;
    resolution=0.2;
}

bool xpLessThan(const QPoint &p1, const QPoint &p2){return p1.x()<p2.x();}
bool ypLessThan(const QPoint &p1, const QPoint &p2){  return p1.y()<p2.y();}

void path::updatemap(QVector<QPoint> mp){
    foreach (QPoint p,mp){
        map.push_back(p);
        search_space.push_back(p);
    }

    qSort(search_space.begin(),search_space.end(),xpLessThan);
    lim.xmax=search_space.last().x();
    lim.xmin=search_space.first().x();
    qSort(search_space.begin(),search_space.end(),ypLessThan);
    lim.ymax=search_space.last().y();
    lim.ymin=search_space.first().y();


    //    modify the local map
        map.push_front(QPoint(lim.xmax,lim.ymin));

}

bool path::input(QPoint robot, QPoint Goal){
    R=robot;
    goal=Goal;
    if (map.isEmpty()) return false;
    if(wn_PnPoly(goal)==-1)
        return true;
    else
        return false;

}

QVector<QPoint> path::output(){
    searchSpace();
    return foundPath;
}

void path::searchSpace(){

    Dstar *dstar = new Dstar();
    list<state> mypath;
    QVector<double> searchX,searchY;
    int count=0;
//    ---------------------------
    //    initialize dstar
        dstar->init(R.x(),R.y(),goal.x(),goal.y());
        for (float i=R.y();i<=lim.ymax;i+=resolution){

            for(float j=lim.xmin;j<=lim.xmax;j+=resolution)
            {
                QPoint p(j,i);
                if(wn_PnPoly(p)==-1){//add cost of each cell
                    dstar->updateCell(j,i,1/pointDist(p,goal));
                    if(pointDist(p,goal)<=0.02 && count<=3)
                    {
                          searchX.push_back(j);
                          searchY.push_back(i);
                          count+=1;
                    }

                }
                if(count>1)break;
            }
        }
        QPoint mG(searchX.first(),searchY.first()); //moderate goal

    //    update goal position
        dstar->updateGoal(mG.x(),mG.y());
        dstar->replan();
        mypath=dstar->getPath();
        foreach(state path,mypath)
            foundPath.push_back(QPoint(path.x,path.y));
}

int path::wn_PnPoly( QPoint P ){
    int    wn = 0;    // the  winding number counter

    bool clock,anticlock;

    // loop through all edges of the polygon
//    clock wise rotation
    foreach(QPoint V,search_space){

        if (V.y() <= P.y()) {
            if (R.y() > P.y()){

                if (isLeft( V, R, P) > 0)
                    ++wn;
            }
        }
        else {                        // start y > P.y (no test needed)
            if (R.y()  <= P.y()){

                 if (isLeft( V, R, P) < 0)  // P right of  edge
                     --wn;            // have  a valid down intersect
            }
        }
       }
    clock=-1*wn;
//    counter clock rotation
    wn=0;
    foreach(QPoint V,search_space){

        if (V.y() <= P.y()) {
            if (R.y() > P.y()){

                if (isLeft( V, R, P) < 0)
                    ++wn;
            }
        }
        else {                        // start y > P.y (no test needed)
            if (R.y()  <= P.y()){

                 if (isLeft( V, R, P) > 0)  // P right of  edge
                     --wn;            // have  a valid down intersect
            }
        }
       }
    anticlock=-1*wn;

    if (anticlock && clock)
        return -1;
    else
        return wn;
}

//general functions description

float path::pointDist(QPoint p, QPoint q){
    return sqrt(pow(p.x()-q.x(),2)+pow(p.y()-q.y(),2));
}
float path::isLeft( QPoint V, QPoint R, QPoint P){
    float position =  (R.x()-V.x())*(P.y()-V.y()) - (R.y()-V.y())*(P.x()-V.x());
    return position;
}
