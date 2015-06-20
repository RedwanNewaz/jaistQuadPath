#include "planner.h"
#include "ui_planner.h"
#include "search.h"
#include "Dstar.h"
#include <QDebug>
#include <QtCore>
#include <cmath>
planner::planner(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::planner)
{
    ui->setupUi(this);
    graphInit();
    resolution=0.2; //map resolution

    ui->xBox->setText("X");
    ui->yBox->setText("Y");
    ui->zBox->setText("Z");

    populateMap();
}

planner::~planner()
{
    delete ui;
}

 void planner::graphInit(){
     line=0;
    // add two new graphs and set their look:
    ui->plot->addGraph();
    ui->plot->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
    ui->plot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue

    ui->plot->addGraph();
    ui->plot->graph(1)->setPen(QPen(Qt::green)); // line color red for second graph

     fermatSpiral1 = new QCPCurve(ui->plot->xAxis, ui->plot->yAxis);
     ui->plot->addPlottable(fermatSpiral1);
     fermatSpiral1->setPen(QPen(Qt::red));
     fermatSpiral1->setBrush(QBrush(QColor(255, 0, 0, 20)));





    // generate some points of data (y0 for first, y1 for second graph):
    QVector<double> x(250), y0(250), y1(250);
    for (int i=0; i<250; ++i)
    {
      x[i] = i;
      y0[i] = qExp(-i/150.0)*qCos(i/10.0); // exponentially decaying cosine
      y1[i] = qExp(-i/150.0);              // exponential envelope
    }
    // configure right and top axis to show ticks but no labels:
    // (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)
    ui->plot->xAxis2->setVisible(true);
    ui->plot->xAxis2->setTickLabels(false);
    ui->plot->yAxis2->setVisible(true);
    ui->plot->yAxis2->setTickLabels(false);
    // make left and bottom axes always transfer their ranges to right and top axes:
    connect(ui->plot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot->yAxis2, SLOT(setRange(QCPRange)));
    // pass data points to graphs:
    ui->plot->graph(0)->setData(x, y0);
    ui->plot->graph(1)->setData(x, y1);
    // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
    ui->plot->graph(0)->rescaleAxes();
    // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
    ui->plot->graph(1)->rescaleAxes(true);
    // Note: we could have also just called ui->plot->rescaleAxes(); instead
    // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
    ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

 void planner::notify(QString msg){
     line++;
     QString no=QString::number(line)+" :";
     QString pre= "\n"+ui->notification->toPlainText();
     ui->notification->setPlainText(no+msg+pre);
 }

 float planner::isLeft( QPoint V, QPoint R, QPoint P){
    float position =  (R.x()-V.x())*(P.y()-V.y()) - (R.y()-V.y())*(P.x()-V.x());
    return position;
}

 int planner::wn_PnPoly( QPoint P )
    {
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




//cost Calculator
/*
 * given point p(x,y)
 *    find the grid resolution for search such that 0.2
 *    find Xmax,Xmin,Ymax,Ymin w.r.t that point inside the SLAM MAP
 * cost APF: 1-{(Xmax+Xmin+Ymax+Ymin)_point/ (Xmax+Xmin+Ymax+Ymin)_map}
 * cost ORIENTATION: is fixed cost and depends on previous position
 * cost Tracking: depends on robot velocity(angular and linear)
 */



//Probabilistic map
/*
 * given SLAM MAP and robot position
 * OUTPUT probabilistic cost of each grid
 *
 * Find the neighbor of robot cell
 *  if neighbor is in SLAM MAP
 *      calculate cost
 *  else
 *      discard that grid by assign large NUMBER
 * move to the minimum cost Cell
 * repeat the procedure
 * connect the path based on shortest path algorithm
 *
 */



bool xLessThan(const QPoint &p1, const QPoint &p2){ return p1.x()<p2.x();}
bool yLessThan(const QPoint &p1, const QPoint &p2){  return p1.y()<p2.y();}
usermap map2vector(QVector<QPoint>localmap){
    usermap map;
    foreach(QPoint p,localmap){
        map.x.push_back(p.x());
        map.y.push_back(p.y());
    }
    return map;
}

int localmapWn( QPoint P, QVector<double> Vx, QVector<double> Vy  )
   {
   int    wn = 0;
   planner *plan;
    QPoint R(0,0);
    QVector<QPoint>lmp;
    qSort(Vx);
    qSort(Vy);
    QPoint XY_max(Vx.first(),Vy.first()),XY_min(Vx.last(),Vy.last());
    lmp.push_back(XY_max);
    lmp.push_back(XY_min);
    foreach(QPoint V,lmp){

       if (V.y() <= P.y()) {
           if (R.y() > P.y()){

               if (plan->isLeft( V, R, P) > 0)
                   ++wn;
           }
       }
       else {                        // start y > P.y (no test needed)
           if (R.y()  <= P.y()){

                if (plan->isLeft( V, R, P) < 0)  // P right of  edge
                    --wn;            // have  a valid down intersect
           }
       }
      }
        return wn;
   }


float pointDist(QPoint p, QPoint q){
    return sqrt(pow(p.x()-q.x(),2)+pow(p.y()-q.y(),2));
}


void planner::populateMap(){
    QPoint map_max,map_min;
        map_max.setX(-100);
        map_max.setY(10);
        map_min.setX(100);
        map_min.setY(10);

//        make a sine wave to indicate the boundary
        //            calculate R
        float R=pointDist(QPoint(0,0),QPoint(-100,10));
        float phi=0;
        for (float theta=0;theta<=2*M_PI;theta+=resolution){
            double x=R*cos(theta);
            double y=R*sin(theta);
            if(x>=-100 && x<=100 && y>10){
                phi+=resolution;
                phi=fmod(phi,2*M_PI);
                y+=sin(phi);

                map.push_back(QPoint(x,y));

            }
        }



}


 void planner::searchSpace(){
    QString msg;
    msg="search\t";
    Dstar *dstar = new Dstar();
    list<state> mypath;
    int count=0;


    QPoint goal(cmd.x,cmd.y);

    qDebug()<<"goal\t"<<goal;
    QVector<double> searchX,searchY;

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
    QVector<double>pathx,pathy;
//    update goal position
    dstar->updateGoal(mG.x(),mG.y());
    dstar->replan();
    mypath=dstar->getPath();
    foreach(state path,mypath){
        pathx.push_back(path.x);
        pathy.push_back(path.y);
    }

    msg+="X= "+QString::number(mG.x())+"\tY= "+QString::number(mG.y());
    notify(msg);
    fermatSpiral1->setData(pathx, pathy);
//    ui->plot->graph(1)->setData(searchX, searchY);
    ui->plot->replot();

}

 void planner:: pointDraw()
{
//    taking user input
     QString x1=ui->xBox->toPlainText();
     QString y1=ui->yBox->toPlainText();
     double center_x=x1.toDouble();
     double center_y=y1.toDouble();
     cmd.x=center_x;
     cmd.y=center_y;

//     draw a line
     QVector<double>x,y;
     x.push_back(center_x-0.12);
     x.push_back(center_x+0.12);
     y.push_back(center_y-0.12);
     y.push_back(center_y+0.12);
     fermatSpiral1->setData(x, y);
     ui->plot->replot();

//     check point in bounded area
     QPoint point(center_x,center_y);
     int i=wn_PnPoly(point);
     qDebug()<<"point "<< point<<"wn\t"<<QString::number(i);


}

 void planner::mapDraw()
{

//SLAM MAP
/*
 * find the robot position
 * find out the Xmax and Xmin
 *      for lowest Ymin in Xmax return YXmax*
 *      for lowest Ymin in Xmin retrun YXmin*
 * group {p_max[Xmax YXmax*], p_min[Xmin YXmin*]}
 * From P_max to P_min
 *      connect all the nodes based on shortest distance algorithm
 *
 */

//    find the robot position
    robot=cmd;
    R.setX(cmd.x);
    R.setY(cmd.y);

//   dont modify real map
    foreach(QPoint p,map)
     search_space.push_back(p);

//    detrmine the area of local map

    qSort(search_space.begin(),search_space.end(),xLessThan);
    lim.xmax=search_space.last().x();
    lim.xmin=search_space.first().x();
    qSort(search_space.begin(),search_space.end(),yLessThan);
    lim.ymax=search_space.last().y();
    lim.ymin=search_space.first().y();

//    modify the local map
    map.push_front(QPoint(lim.xmax,lim.ymin));

//    left most point
    QPoint left(lim.xmin,lim.ymin);
    QPoint right(lim.xmax,lim.ymin);
    QPoint start(0,0);
    QVector<QPoint>blueMap;

    blueMap.push_back(left);

    blueMap.push_back(start);

    blueMap.push_back(right);

    qDebug()<<left<<right<<start;



   usermap blueline=map2vector(blueMap);

//     connect curve by green color
    usermap slam=map2vector(map);
    ui->plot->graph(0)->setData(blueline.x, blueline.y);
    ui->plot->graph(1)->setData(slam.x, slam.y);// green line for map
    ui->plot->graph(1)->rescaleAxes(true);

    ui->plot->replot();


}

 void planner::on_mapDraw_clicked()
{
    QString x=ui->xBox->toPlainText();
    QString y=ui->yBox->toPlainText();
    QString z=ui->zBox->toPlainText();
    cmd.x=x.toDouble();
    cmd.y=y.toDouble();
    cmd.z=z.toDouble();
    mapDraw();
}

 void planner::on_plannerButton_clicked()
{
    QString x=ui->xBox->toPlainText();
    QString y=ui->yBox->toPlainText();
    QString z=ui->zBox->toPlainText();
    cmd.x=x.toDouble();
    cmd.y=y.toDouble();
    cmd.z=z.toDouble();
}

 void planner::on_pointDraw_clicked()
{
    pointDraw();

    if(wn_PnPoly(QPoint(cmd.x,cmd.y))!=-1){
        notify("Error!!!Invalid goal");
        return;}
    else{
        notify("given\t"+QString::number(cmd.x)+"\t"+ QString::number(cmd.y));

        searchSpace();}
}
