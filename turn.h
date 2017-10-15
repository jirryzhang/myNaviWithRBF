#ifndef TURN_H
#define TURN_H

#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QDir>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "data.h"


class turn
{
    public:
        turn();
        virtual ~turn();
        double **generateTurnNarrow();    /* 行距小于转弯半径 */
        double **generateTurnWide();      /* 行间距大于转弯半径 */
        double **genTurn(double angArg, double xsArg, double ysArg);
        double **segmentation(double arg[3][5]);

        int turn_flag;
        double width;
        double rmin;
        double ang;
        double timestep;
        double xs, ys;

        int countnum;    //计算分割点的总数
        double speed;    //拖拉机行进速度；
        bool narrow;      //行间距与转弯半径的关系。

        QFile *fname;
        QString filename;

    protected:

    private:
};

#endif // TURN_H
