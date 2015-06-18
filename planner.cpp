#include "planner.h"
#include "ui_planner.h"
#include "search.h"
#include <QDebug>
#include <QtCore>
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

    QPoint map_max,map_min;
        map_max.setX(-100);
        map_max.setY(10);
        map_min.setX(100);
        map_min.setY(10);
    map.push_back(map_max);
    map.push_back(map_min);
}

planner::~planner()
{
    delete ui;
}

 void planner::graphInit(){
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

 float planner::isLeft( QPoint V, QPoint R, QPoint P){
    float position =  (R.x()-V.x())*(P.y()-V.y()) - (R.y()-V.y())*(P.x()-V.x());
    return position;
}

 int planner::wn_PnPoly( QPoint P )
    {
    int    wn = 0;    // the  winding number counter


    // loop through all edges of the polygon
    foreach(QPoint V,map){

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

 void planner::searchSpace(){
    QString msg;
    msg=ui->notification->toPlainText()+"\n"+"search\t";

    int count=0;
    bool loopbreak=false;
    float dist,min=00;
    QPoint goal(cmd.x,cmd.y);

    for (float i=R.y();i<=lim.ymax;i+=resolution){

        for(float j=lim.xmin;j<=lim.xmax;j+=resolution)
        {
            QPoint p(i,j);
            if(wn_PnPoly(p)==-1){
                count+=1;
                search_space.push_back(p);
//                dist=(p.x()-goal.x())*(p.x()-goal.x())+(p.y()-goal.y())*(p.y()-goal.y());
//                if (dist>=1){
//                    min=dist;

//                    msg+="\t found \t";
//                    qDebug()<<QString::number(dist);
//                    loopbreak=true;
//                    break;
//                }
            }

        }
        if(loopbreak)
            break;

    }
    msg+=QString::number(count)+"\t"+QString::number(min);
    ui->notification->setPlainText(msg);
    qDebug()<<search_space.size();
}

 void planner:: pointDraw()
{
//    taking user input
     QString x1=ui->xBox->toPlainText();
     QString y1=ui->yBox->toPlainText();
     double center_x=x1.toDouble();
     double center_y=y1.toDouble();;

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
     ui->notification->setPlainText("wn\t"+QString::number(i));


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

//    detrmine the area of local map
    qSort(map.begin(),map.end(),xLessThan);
    qSort(map.begin(),map.end(),yLessThan);
    lim.xmax=map.last().x();
    lim.xmin=map.first().x();
    lim.ymax=map.last().y();
    lim.ymin=map.first().y();

//    Draw the local map
    QVector<double> x,y;
    x.push_back(lim.xmax);
    x.push_back(R.x());
    x.push_back(lim.xmin);

    y.push_back(lim.ymax);
    y.push_back(R.y());
    y.push_back(lim.ymin);

//     connect curve by red color
    usermap slam=map2vector(map);
    ui->plot->graph(0)->setData(x, y);
    ui->plot->graph(1)->setData(slam.x, slam.y);// red line for map
    ui->plot->graph(0)->rescaleAxes(true);

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
    searchSpace();
}
