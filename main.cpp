#include <QCoreApplication>
#include "serial.h"
#include "data.h"
#include <QTextStream>
#include "recvthread.h"
#include "recvthreadcan.h"


extern std::ofstream fpid;

/* 开启设备can0的命令,并设置波特率800k */
#define COMMAND "/sbin/ip link set can0 type can bitrate 800000"
#define UP "ifconfig can0 up"
#define DOWN "ifconfig can0 down"

/* 开启设备can1的命令,并设置波特率800k */
#define COMMAND1 "/sbin/ip link set can1 type can bitrate 125000"
#define UP1 "ifconfig can1 up"
#define DOWN1 "ifconfig can1 down"
#define H 2

/* 初始化data类静态变量的数据 */
double data::GPStime = 0;
double data::lat=0;
double data::lng=0;
char data::satelite_num[16]="0";
int  data::num=0;
double data::altitude=0;
double data::north=0;
double data::dnorth = 0;
double data::velocity=0;
int data::status=99999;
double data::theta=0;
double data::car_length=0;

double data::distance=0;
double data::distance_mid = 0;
double data::calculFai=0;
double data::rolldegree = 0;

double data::AX_coor=0;
double data::AX_coor_mid = 0;
double data::AY_coor=0;
double data::AY_coor_mid = 0;
double data::BX_coor=0;
double data::BX_coor_mid = 0;
double data::BY_coor=0;
double data::BY_coor_mid = 0;
double data::x_mid = 0;
double data::y_mid = 0;
double data::xInit = 0;
double data::yInit = 0;
int data::motor_vel = 0;

int data::turnFlag = 0;
double data::velValue = 0;
double data::inValue = 0;

double data::p1x = 0;
double data::p1y = 0;
double data::p2x = 0;
double data::p2y = 0;
double data::Kd = 0;
double data::Ki = 0;
double data::Kp = 0;
int data::algorithmchose = 0;

char recvthread::blueSdata[256] = "this is test\n";

int data::flagTemp   =0;
int data::ABturnFlag = 0;
double data::expectedCircle = 0;
double data::diffTheta      = 0;
double data::initTheta      = 79.10;
double data::slopeAB        = 0;
double data::xaMid_coor     = 0;
double data::yaMid_coor     = 0;
double data::udiffTheta     = 0;
double data::udistanceMid   = 0;
double data::rollAngle      = 0;
double data::pitchAngle     = 0;

int orderFlag=0;
/* 接受上位机的命令的对象 */
recvthread *recvT =new recvthread();
recvthreadcan *recvSensor = new recvthreadcan();

/* 处理接受上位机设置命令的函数 */
void orderhandle() {

    QFile *fABname = new QFile("/opt/GPS.txt");
    QTextStream out(fABname);
    /* 地头直线 */
    QFile *farm = new QFile("/opt/farm.txt");
    QTextStream outfarm(farm);

    switch (orderFlag) {
    printf("the recieved order = %d", orderFlag);

    /* start pilot */
    case 10000:
        /* start pilot */
        recvT->pilotalg->tracontrol->initPara();
        recvT->pilotflag = 1;
        fpid.open("data/pid.txt");
        //recvT->pilotalg->tracontrol->initPara();
        break;
    case 20000:
        /* stop pilot */
        recvT->pilotflag = 2;
        fpid.close();
        break;
        /* 设置AB点的指令 */
    case 40003:

        fABname->open(QIODevice::ReadWrite | QIODevice::Text);

        data::AX_coor = out.readLine().toDouble();
        data::AY_coor = out.readLine().toDouble();
        data::BX_coor = out.readLine().toDouble();
        data::BY_coor = out.readLine().toDouble();
        data::AX_coor_mid = out.readLine().toDouble();
        data::AY_coor_mid = out.readLine().toDouble();
        data::BX_coor_mid = out.readLine().toDouble();
        data::BY_coor_mid = out.readLine().toDouble();

        fABname->close();

        data::AX_coor = data::lat;
        data::AY_coor = data::lng;
//        data::AX_coor_mid= data::AX_coor - 0.9*cos((data::north-90)/180*M_PI) + H*sin(data::rolldegree/180*M_PI);
//        data::AY_coor_mid= data::AY_coor - 0.9*sin((data::north-90)/180*M_PI);
        data::AX_coor_mid= data::AX_coor - 0.9*cos((data::north-90)/180*M_PI);
        data::AY_coor_mid= data::AY_coor - 0.9*sin((data::north-90)/180*M_PI);

        fABname->open(QIODevice::ReadWrite | QIODevice::Text);

        out << QString::number(data::AX_coor,'g',12) <<endl;
        out << QString::number(data::AY_coor,'g',12) <<endl;
        out << QString::number(data::BX_coor,'g',12) <<endl;
        out << QString::number(data::BY_coor,'g',12) <<endl;
        out << QString::number(data::AX_coor_mid,'g',12) <<endl;
        out << QString::number(data::AY_coor_mid,'g',12) <<endl;
        out << QString::number(data::BX_coor_mid,'g',12) <<endl;
        out << QString::number(data::BY_coor_mid,'g',12) <<endl;

        fABname->close();

        data::status = 40103;
        printf("the status is %d.\n",data::status);

        break;

    case 40004:

        fABname->open(QIODevice::ReadWrite | QIODevice::Text);

        data::AX_coor = out.readLine().toDouble();
        data::AY_coor = out.readLine().toDouble();
        data::BX_coor = out.readLine().toDouble();
        data::BY_coor = out.readLine().toDouble();
        data::AX_coor_mid = out.readLine().toDouble();
        data::AY_coor_mid = out.readLine().toDouble();
        data::BX_coor_mid = out.readLine().toDouble();
        data::BY_coor_mid = out.readLine().toDouble();
        fABname->close();

        data::BX_coor = data::lat;
        data::BY_coor = data::lng;
       //data::BX_coor_mid= data::BX_coor - 0.9*cos((data::north-90)/180*M_PI) + H*sin(data::rolldegree/180*M_PI);
       //data::BY_coor_mid= data::BY_coor - 0.9*sin((data::north-90)/180*M_PI);
        data::BX_coor_mid= data::BX_coor - 0.9*cos((data::north-90)/180*M_PI);
        data::BY_coor_mid= data::BY_coor- 0.9*sin((data::north-90)/180*M_PI);

        fABname->open(QIODevice::ReadWrite | QIODevice::Text);

        out << QString::number(data::AX_coor,'g',12) <<endl;
        out << QString::number(data::AY_coor,'g',12) <<endl;
        out << QString::number(data::BX_coor,'g',12) <<endl;
        out << QString::number(data::BY_coor,'g',12) <<endl;
        out << QString::number(data::AX_coor_mid,'g',12) <<endl;
        out << QString::number(data::AY_coor_mid,'g',12) <<endl;
        out << QString::number(data::BX_coor_mid,'g',12) <<endl;
        out << QString::number(data::BY_coor_mid,'g',12) <<endl;

        fABname->close();

        data::status = 40104;
        printf("hello 40004.\n");
        printf("the status is %d.\n",data::status);
        break;

        /* 转弯的控制指令 */
    case 30001:
        /* 右转 */
        data::turnFlag   = 1;
        data::ABturnFlag = 1;
//        data::status = 10020;
        printf("the turn right!\n");
        break;
    case 30002:
        /* 左转 */
        data::turnFlag   = 2;
        data::ABturnFlag = 2;  //目标AB线斜率的标记
//        data::status = 10030;
        printf("the turn left!\n");
        break;
    case 40013:
        /* A点设置成功 */
        data::status = 99999;
        break;
    case 40014:
        /* B点设置成功 */
        data::status = 99999;
        break;
    case 50001:

        farm->open(QIODevice::ReadWrite | QIODevice::Text);
        data::p1x = outfarm.readLine().toDouble();
        data::p1y = outfarm.readLine().toDouble();
        data::p2x = outfarm.readLine().toDouble();
        data::p2y = outfarm.readLine().toDouble();

        farm->close();

        data::p1x= data::lat - 0.9*cos((data::north-90)/180*M_PI) + H*sin(data::rolldegree/180*M_PI);
        data::p1y= data::lng - 0.9*sin((data::north-90)/180*M_PI);

        farm->open(QIODevice::ReadWrite | QIODevice::Text);
        outfarm << QString::number(data::p1x,'g',12) <<endl;
        outfarm << QString::number(data::p1y,'g',12) <<endl;
        outfarm << QString::number(data::p2x,'g',12) <<endl;
        outfarm << QString::number(data::p2y,'g',12) <<endl;

        farm->close();
        data::status = 50101;
        printf("the status = %d\n", data::status);
        break;

    case 50002:

        farm->open(QIODevice::ReadWrite | QIODevice::Text);
        data::p1x = outfarm.readLine().toDouble();
        data::p1y = outfarm.readLine().toDouble();
        data::p2x = outfarm.readLine().toDouble();
        data::p2y = outfarm.readLine().toDouble();

        farm->close();
        data::p2x= data::lat - 0.9*cos((data::north-90)/180*M_PI) + H*sin(data::rolldegree/180*M_PI);
        data::p2y= data::lng - 0.9*sin((data::north-90)/180*M_PI);

        farm->open(QIODevice::ReadWrite | QIODevice::Text);
        outfarm << QString::number(data::p1x,'g',12) <<endl;
        outfarm << QString::number(data::p1y,'g',12) <<endl;
        outfarm << QString::number(data::p2x,'g',12) <<endl;
        outfarm << QString::number(data::p2y,'g',12) <<endl;

        farm->close();
        data::status = 50102;
        printf("the status = %d\n", data::status);
        break;

    case 50011:
        data::status = 99999;
        break;
    case 50012:
        data::status = 99999;
        break;
    }

}
/* 主函数 */
int main()
{
	/* 使能CAN设备 */
    system(DOWN);
    system(COMMAND);
    system(UP);

    system(DOWN1);
    system(COMMAND1);
    system(UP1);

    char recvBlue[32]="";

    if ( (recvT->fBlue = recvT->serialBlue->open_port(recvT->fBlue, 3)) < 0 ) {
        perror("open_port fBlue error!");
    }
    if (( recvT->serialBlue->set_opt(recvT->fBlue, 19200, 8, 'N', 1)) < 0 ) {
        perror("set port fBlue error!");
    }

    while(1) {
        recvT->numBlue = 0;
        memset(recvBlue,0,sizeof(recvBlue));
        while( (recvT->numBlue = recvT->serialBlue->Recv(recvT->fBlue, recvBlue, 32)) > 0) {
            /* 处理接受的命令
            通信正常，即开始接受天线数据*/
//            if (recvT->isFinished()) {

//                recvT->start();
//            }

            if (!(recvT->isRunning())) {
                printf("this is running. \n");
                recvT->start();

            }
            /* 倾角传感器线程的开启　*/
            if (!(recvSensor->isRunning())) {
                recvSensor->start();
            }

            recvT->data_handle(recvBlue);

//            if(orderFlag != data::flagTemp) {
                orderFlag = data::flagTemp;
                orderhandle();
//            }
//            printf("recive OK!the numblue = %d\n",recvT->numBlue);
        }
		/* 通信中断,停止导航等相关操作 */
        if (recvT->numBlue == 0) {
            /* stop the motor and the recieve data */
            printf("Don't recieve the command!\n");
            /* 结束卫星定位线程　*/
            if( recvT->isRunning() ) {
                recvT->stop();
                orderFlag = 0;
            }
            /* 结束传感器线程　*/
            if (recvSensor->isRunning()) {
                recvSensor->stop();
            }
        }
    }

    recvT->serialBlue->Close(recvT->fBlue);

    return 0;
}
