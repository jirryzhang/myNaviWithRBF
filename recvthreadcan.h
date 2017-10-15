/*
 * recvthread.h
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#ifndef RECVTHREADCAN_H_
#define RECVTHREADCAN_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <qthread.h>

#include "can01.h"
#include "can00.h"
#include "data.h"

#include <QDebug>


class recvthreadcan: public QThread {
    Q_OBJECT
public:
    recvthreadcan();
    virtual ~recvthreadcan();
    void run();
    void stop();   
    double frametempDataTodouble(unsigned char *can_data);


signals:
    void send_can();

private:

    can01 *canrecv;
    int numCan;
    struct can_frame frametemp;
    int BCDToInt(unsigned char m);

    volatile bool stopped;

private slots:

};

#endif /* RECVTHREADCAN_H_ */
