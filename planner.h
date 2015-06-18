#ifndef PLANNER_H
#define PLANNER_H

#include <QMainWindow>
#include "qcustomplot.h"

typedef struct {
    QVector<double>x,y;
}usermap;



namespace Ui {
class planner;
}

class planner : public QMainWindow
{
    Q_OBJECT

public:
    explicit planner(QWidget *parent = 0);
    ~planner();
    void searchSpace();



private slots:

    void graphInit();

    void mapDraw();

    void on_mapDraw_clicked();

    void on_plannerButton_clicked();

    void pointDraw();

    int wn_PnPoly( QPoint );


    void on_pointDraw_clicked();

    float isLeft( QPoint V, QPoint R, QPoint P);


private:
    Ui::planner *ui;
    QCPCurve *fermatSpiral1;

    struct user{
        double x;
        double y;
        double z;
    }cmd,robot;
    struct property{
        double xmin,xmax;
        double ymin,ymax;
    }lim;
    double resolution;






    QVector<QPoint> map,search_space;
    QPoint R;


};

#endif // PLANNER_H
