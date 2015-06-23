#include "stack.h"
//https://github.com/KHEngineering/SmoothPathPlanner
planner::planner(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::planner)
{
    ui->setupUi(this);
    graphInit();
    resolution=0.2; //map resolution
    interval=1;

    plan=new path;

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

     //viuslaize robot triangle instead of trajectory
     trajectoryLine = new QCPCurve(ui->plot->xAxis, ui->plot->yAxis);
     ui->plot->addPlottable(trajectoryLine);
     trajectoryLine->setPen(QPen(Qt::black));
     trajectoryLine->setBrush(QBrush(QColor(0, 0, 0, 20)));

     // generate some points of data (y0 for first, y1 for second graph):
    QVector<double> x(250), y0(250), y1(250);
    for (int i=0; i<250; ++i)
    {
      x[i] = i;
      y0[i] = qExp(-i/150.0)*qCos(i/10.0); // exponentially decaying cosine
      y1[i] = qExp(-i/150.0);              // exponential envelope
    }
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
    ui->plot->graph(0)->rescaleAxes();
    ui->plot->graph(1)->rescaleAxes(true);
    ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

 void planner::notify(QString msg){
     line++;
     QString no=QString::number(line)+" :";
     QString pre= "\n"+ui->notification->toPlainText();
     ui->notification->setPlainText(no+msg+pre);
 }

 usermap map2vector(QVector<QPoint>localmap){
    usermap map;
    foreach(QPoint p,localmap){
        map.x.push_back(p.x());
        map.y.push_back(p.y());
    }
    return map;
}


 void planner::populateMap(){
    QPoint map_max,map_min;
        map_max.setX(-100);
        map_max.setY(10);
        map_min.setX(100);
        map_min.setY(10);

//        make a sine wave to indicate the boundary
        //            calculate R
        float R=plan->pointDist(QPoint(0,0),QPoint(-100,10));
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

 void planner::robotView(QPointF p){
     double r=1;
     double rot=(M_PI/360)*(60+180);
     QVector<double>x,y;
     for(float theta=2*M_PI+rot;theta>=rot;theta-=2*M_PI/3)
     {
         x.push_back(p.x()+r*cos(theta));
         y.push_back(p.y()+r*sin(theta));
     }
     x.push_front(p.x()+r*cos(2*M_PI+rot));
     y.push_front(p.y()+r*sin(2*M_PI+rot));

     trajectoryLine->setData(x, y);
     ui->plot->replot();
 }

 void planner::searchSpace(){
    QString msg;
    msg="search\t";
    QPoint G(cmd.x,cmd.y);
    robotView(G);

    bool result=plan->input(R,G);
    if(!result){
        msg+="result not found!!!";
        notify(msg);
        return;
    }
    search_space=plan->output();
    usermap planpath=map2vector(search_space);
    fermatSpiral1->setData(planpath.x, planpath.y);
    ui->plot->replot();


//    path smoothing
    pathsmoother *smooth;
    smooth =new pathsmoother;
    int dirChange=smooth->DirChange(search_space);


    QVector<double>pathx,pathy;
    mutex.lock();
    foreach (QPointF p, smooth->splinePath()){
        pathx.push_back(p.x());
        pathy.push_back(p.y());
        robotView(p);
//        QThread::msleep(interval/3);
    }
    mutex.unlock();

    trajectoryLine->setData(pathx, pathy);
    ui->plot->replot();


    msg+="result "+QString::number(dirChange);

    notify(msg);



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
//     int i=wn_PnPoly(point);
//     qDebug()<<"point "<< point<<"wn\t"<<QString::number(i);


}

 void planner::mapDraw()
{

//    find the robot position

    robot=cmd;
    R.setX(cmd.x);
    R.setY(cmd.y);

//   dont modify real map
    plan->updatemap(map);

//    modify the local map
    map.push_front(QPoint(plan->lim.xmax,plan->lim.ymin));

//    left most point
    QPoint left(plan->lim.xmin,plan->lim.ymin);
    QPoint right(plan->lim.xmax,plan->lim.ymin);
    QPoint start=R;
    QVector<QPoint>blueMap;

    blueMap.push_back(left);

    blueMap.push_back(start);

    blueMap.push_back(right);

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


 void planner::on_pointDraw_clicked()
{
    pointDraw();
    searchSpace();
}

void planner::on_reso_valueChanged(int value)
{
    QString my("animation interval "+QString::number(value));
    interval=value;
    ui->antime->setText(my);
}
