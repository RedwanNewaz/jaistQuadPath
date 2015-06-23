#ifndef PLANNER_H
#define PLANNER_H

#include <QMainWindow>
#include "qcustomplot.h"
#include "path.h"

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

    void populateMap();

    void robotView(QPointF);

    path *plan;


private slots:

    void graphInit();

    void notify(QString);

    void mapDraw();

    void on_mapDraw_clicked();

    void pointDraw();

    void on_pointDraw_clicked();




    void on_reso_valueChanged(int value);

private:
    Ui::planner *ui;
    QMutex mutex;
    QCPCurve *fermatSpiral1, *trajectoryLine;
    float interval;
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
    int line;






    QVector<QPoint> map,search_space;
    QPoint R;


};

#endif // PLANNER_H
