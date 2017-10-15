#ifndef RECVTHREAD_H_
#define RECVTHREAD_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <qthread.h>

#include "serial.h"
#include "algorithm.h"
#include "data.h"


#include <QDir>
#include <QTextStream>
#include <QFile>
#include <QDateTime>
#include <QDebug>


class recvthread: public QThread {
    Q_OBJECT
public:
    recvthread();
    virtual ~recvthread();
    void run();
    void data_handle(char data[]);
    double* change(double B,double L);
    void stop();

    double sysUsecTime();
    double degree;
    int pilotflag;

    serial *serialBlue;
    int numBlue;
    int fBlue;

    static char blueSdata[256];

    /* 保存数据的文件 */
    QFile *fnameGPS;
    QString filename;

private:
    volatile bool stopped;

    /* serial settings */
    serial *serialGPS;
    serial *serial485;
    int fGPS;
    int f485;
    int num485;
    int numGPS;

    algorithm *pilotalg;

};

#endif /* RECVTHREAD_H_ */
