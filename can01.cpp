/*
 * can01.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#include "can01.h"

can01::can01() {
    // TODO Auto-generated constructor stub
}
/* 整数转换成16进制数 */
void can01::intToHex(char start[], int n){ //把步速n转化成电机能接受的指令start[4]
    int int_byte_change[8];
    int xx=abs(n);
    int_byte_change[0]= xx/16/16/16/16/16/16/16;
    int_byte_change[1]= xx/16/16/16/16/16/16%16;
    int_byte_change[2]= xx/16/16/16/16/16%16%16;
    int_byte_change[3]= xx/16/16/16/16%16%16%16;
    int_byte_change[4]= xx/16/16/16%16%16%16%16;
    int_byte_change[5]= xx/16/16%16%16%16%16%16;
    int_byte_change[6]= xx/16%16%16%16%16%16%16;
    int_byte_change[7]= xx%16%16%16%16%16%16%16;
    start[0] = int_byte_change[6]*16+int_byte_change[7];
    start[1] = int_byte_change[4]*16+int_byte_change[5];
    start[2] = int_byte_change[2]*16+int_byte_change[3];
    start[3] = int_byte_change[0]*16+int_byte_change[1];
    if(n<0){
        start[3] = 0xFF-start[3];
        start[2] = 0xFF-start[2];
        start[1] = 0xFF-start[1];
        start[0] = 0xFF-start[0]+0x01;
    }
}


can01::~can01() {
    // TODO Auto-generated destructor stub
}
/* 带有过滤规则的接受CAN节点数据 */
int can01::can_recv(struct can_frame frame[])
{
    int s=0;
    unsigned long nbytes = 0;
    struct sockaddr_can addr;	//can
    struct ifreq ifr;	//
    //struct can_frame frame;	//存储发送的数据信息的结构体
    struct can_filter rfilter[5];	//过滤规则的结构体
	/* 创建套接字 */
    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)	{
        perror("Error while opening socket");
        //return -1;
    }

    strncpy(ifr.ifr_name, "can1", sizeof(ifr.ifr_name)-1);
    ifr.ifr_name[sizeof(ifr.ifr_name)-1] = '\0';
    ioctl(s, SIOCGIFINDEX, &ifr);	//指定can0设备

    addr.can_family = AF_CAN;	//这两行必须有
    addr.can_ifindex =0;

    if((bind(s, (struct sockaddr *)&addr, sizeof(addr))) < 0)
    {
        perror("Error in socket bind");
        //return -2;
    }
    /*只有扩展帧能通过，CAN_SFF_MASK 是标准帧，
     * CAN_SFF_MASK & CAN_EFF_MASK, 都能通过。
     */

    rfilter[0].can_id = 0x00000585;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = 0x00200002;
    rfilter[1].can_mask = CAN_EFF_MASK;

	
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));	//设置过滤规则
    nbytes = read(s, frame, sizeof(*frame));
	
    close(s);

    return(nbytes);
}

void can01::can_send(int a){

    int s = 0, nbytes= 0;

    struct sockaddr_can addr;
    struct ifreq ifr;		//s,addr,ifr都是为了将嵌套字和can接口连接
    struct can_frame frame;	//存储数据信息

    s = socket(PF_CAN,SOCK_RAW,CAN_RAW);	//创建一个套接字

    strncpy(ifr.ifr_name,"can1", sizeof(ifr.ifr_name)-1);
    ifr.ifr_name[sizeof(ifr.ifr_name)-1] = '\0';
    ioctl(s,SIOCGIFINDEX,&ifr);			//指定CAN0设备,得到索引

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s,(struct sockaddr*)&addr,sizeof(addr));//将套接字与can0设备连接在一起

    /* set the motor speed */
    frame.can_id = CAN_EFF_FLAG|0x05000087;      //设置电机的脉冲数指令
    frame.can_dlc = 4;

    char start[4];
    intToHex(start,a);

    for(int i=0;i<4;i++) {
        frame.data[i] =start[i];
    }
    nbytes = write(s, &frame, sizeof(frame));
    /* enable the motor */
    frame.can_id = CAN_EFF_FLAG|0x05000081;     //使能电机指令
    frame.can_dlc = 0;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    nbytes = write(s, &frame, sizeof(frame));
    /* stop the motor */
    if(a == 99999) {
        frame.can_id = CAN_EFF_FLAG|0x05000082;  //电机脱机指令
        frame.can_dlc = 0;
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
        nbytes = write(s, &frame, sizeof(frame));
    }

    close(s);

}
/*控制前进速度的函数*/
void can01::velocityCan(int v)
{
    int s = 0, nbytes= 0;
    struct sockaddr_can addr;
    struct ifreq ifr;		//s,addr,ifr都是为了将嵌套字和can接口连接
    struct can_frame frame;	//存储数据信息
    //struct cna_filter rfilter;	//过滤规则
    s = socket(PF_CAN,SOCK_RAW,CAN_RAW);	//创建一个套接字

    strncpy(ifr.ifr_name,"can1", sizeof(ifr.ifr_name)-1);
    ifr.ifr_name[sizeof(ifr.ifr_name)-1] = '\0';
    ioctl(s,SIOCGIFINDEX,&ifr);			//指定CAN0设备,得到索引

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s,(struct sockaddr*)&addr,sizeof(addr));//将套接字与can0设备连接在一起

    frame.can_id = CAN_EFF_FLAG|0x00200001;
    frame.can_dlc = 1;
	/* 整数转化为16进制数 */
    int byte[2];
    byte[1]=v%16;
    byte[0]=v/16;
    char dataHex[1];
    dataHex[0] = byte[0]*16 + byte[1];

    frame.data[0] = dataHex[0];

    nbytes = write(s, &frame, sizeof(frame));

    close(s);
}
