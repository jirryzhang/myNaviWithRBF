/*
 * can0.h
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#ifndef CAN00_H_
#define CAN00_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/ioctl.h>	//这四条是can通信的头文件
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <signal.h>
#include <unistd.h>
/* linux 文件的读写头文件 */
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <QDebug>

#ifndef PF_CAN			//条件编译
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#define CAN_EFF_FLAG 0x80000000U	//扩展帧的标识
#endif
class can00 {
public:
    can00();
    virtual ~can00();
    int can_recv(struct can_frame frame[]);  //接受can数据
    void can_send(int a);                    //发送can数据，当接受值从0-8代表不同发送can指令
    void velocityCan(int v);                 //发送控制速度的程序

    void intToHex(char start[], int n);
};

#endif /* CAN00_H_ */
