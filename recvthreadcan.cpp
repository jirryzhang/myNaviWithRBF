/*
 * recvthreadcan.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#include "recvthreadcan.h"

recvthreadcan::recvthreadcan() {


    stopped=false;
    canrecv = new can01();
    numCan=0;
}

recvthreadcan::~recvthreadcan() {
    // TODO Auto-generated destructor stub
}
void recvthreadcan::run(){

    while(!stopped) {
        /* 改变接收方式
          CAN为阻塞的接受方式
        */

        numCan = canrecv->can_recv(&frametemp);
//        printf("the recvsensor is running!\n");


        if(frametemp.can_id == 0x585){

            //北微传感器安装方式：传感器Y轴与车头前进方向重合
            data::rollAngle = frametemp.data[1]/16*10 + frametemp.data[1]%16 + (frametemp.data[2]/16*10+frametemp.data[2]%16)*0.01 + (frametemp.data[3]/16*10+frametemp.data[3]%16)*0.01*0.01;
            /* X轴 */
            if (frametemp.data[0]==0x10)
                data::rollAngle = -data::rollAngle;
            /* Ｙ轴 */
            data::pitchAngle = frametemp.data[5]/16*10 + frametemp.data[5]%16 + (frametemp.data[6]/16*10+frametemp.data[6]%16)*0.01 + (frametemp.data[7]/16*10+frametemp.data[7]%16)*0.01*0.01;
            if (frametemp.data[4]==0x10)
                data::pitchAngle = -data::pitchAngle;
            qDebug()<<"left:"<<data::rolldegree;

            qDebug()<<"right:"<<data::pitchAngle;
        }
    }

    stopped=false;
}


/* shutdown the serial */
void recvthreadcan::stop(){

    stopped=true;
}

int recvthreadcan::BCDToInt(unsigned char m) {
    int a=0,b=1;
    unsigned char n;
    while(m) {
        n=0xF&m;
        m=m>>4;
        a=a+b*n;
        b=b*10;
    }
    return a;
}

double recvthreadcan::frametempDataTodouble(unsigned char *can_data) {
    int z,a, b, c;
    //    int b,c;
    double d;
    z = BCDToInt(can_data[4]);
    a = BCDToInt(can_data[5]);
    b = BCDToInt(can_data[6]);
    c = BCDToInt(can_data[7]);

    if(z>=10) {
        //        d = -((a-10)*100.0+b+c*0.01);
        d = -(a+ b*0.01+c*0.0001);
    } else {
        d = a +0.01*b+c*0.0001;
    }

    return d;
}
