/*
 * recvthread.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#include "recvthread.h"
#define H 2

recvthread::recvthread() {

    degree=0;
    stopped=false;
    serial485=new serial();
    serialGPS=new serial();

    f485=0, fGPS=0;

    serialBlue = new serial();
    fBlue=0;

    pilotalg = new algorithm();
    pilotflag = 0;
    /* 创建保存数据的文件 */
    int filenum = 0;
    filename = "data/"+QString::number(filenum) + "gps.txt";
    fnameGPS = new QFile(filename);

    while(fnameGPS->exists()) {
        filenum++;
        filename = "data/" + QString::number(filenum) + "gps.txt";
        fnameGPS = new QFile(filename);
    }
}

recvthread::~recvthread() {
    // TODO Auto-generated destructor stub
}
void recvthread::run(){

    char recv_buff[128] = "0";
    char buff[]="0";

    // set the GPS
    if((fGPS=serialGPS->open_port(fGPS,2))<0) {
        perror("open_port error");
    }
    // 设置串口的波特率，校验位等
    if(serialGPS->set_opt(fGPS,115200,8,'N',1)<0) {
        perror("set_opt error");
    }
    //  set the RS485
    if((f485=serial485->open_port(f485,4))<0) {
        perror("open_port error");
    }
    if((serial485->set_opt(f485,9600,8,'E',1))<0) {
        perror("set_opt error");
    }
    /* 注释的数据 */
#if 0
    /* open and set the blue */
    if ( (fBlue = serialBlue->open_port(fBlue, 3)) < 0 ) {
        perror("open_port fBlue error!");
    }

    if (( serialBlue->set_opt(fBlue, 19200, 8, 'N', 1)) < 0 ) {
        perror("set port fBlue error!");
    }
#endif

    printf("the stopped = %d\n", stopped);
    while(!stopped) {

        numGPS = 0, num485 = 0;
        memset(recv_buff,0,sizeof(recv_buff));
        memset(buff,0,sizeof(buff));

        numBlue = 0;
        memset(blueSdata, 0, sizeof(blueSdata));

        /* 角度传感器的数据 */
        while((num485=serial485->Recv(f485,buff,8)) >0) {

            degree=serial485->buffToDouble(buff);

            if(degree >= 180) {
                degree=-degree+360;
            } else {
                degree=-degree;
            }
            data::theta=degree;
            //            printf("the degree = %f. \n",degree);
        }
        //        printf("the pilotflag = %d. \n",pilotflag);

        while((numGPS = serialGPS->Recv(fGPS, recv_buff, 128)) > 0) {


            if((strstr(recv_buff, "GPGGA") != NULL) ) {
                data_handle(recv_buff);


                //data::x_mid= data::lat - 0.9*cos((data::north-90)/180*M_PI) + H*sin(data::rolldegree/180*M_PI);
                //data::y_mid= data::lng - 0.9*sin((data::north-90)/180*M_PI);
                /* 为添加倾角传感起的定位坐标　*/
//                data::x_mid= data::lat - 0.9*cos((data::north-90)/180*M_PI);
//                data::y_mid= data::lng - 0.9*sin((data::north-90)/180*M_PI);
                double pitchRad = data::pitchAngle/180*M_PI;
                double rollRad = data::rollAngle/180*M_PI;
                double northRad = data::north/180*M_PI;

                /* 添加横滚角的定位坐标　*/
                data::xInit= data::lat - 0.9*cos((data::north-90)/180*M_PI);
                data::yInit= data::lng - 0.9*sin((data::north-90)/180*M_PI);

//                data::x_mid= data::lat - 0.9*cos((data::north-90)/180*M_PI) - H*sin(rollRad)*sin(northRad);
//                data::y_mid= data::lng - 0.9*sin((data::north-90)/180*M_PI) + H*sin(rollRad)*cos(northRad);
//                /* 添加横滚角和俯仰角的定位坐标　*/
                data::x_mid= data::lat - 0.9*cos((data::north-90)/180*M_PI) - H*sin(rollRad)*sin(northRad) + H*sin(pitchRad)*cos(northRad);
                data::y_mid= data::lng - 0.9*sin((data::north-90)/180*M_PI) + H*sin(rollRad)*cos(northRad) + H*sin(pitchRad)*sin(northRad);

                if (pilotflag == 1) {
                    pilotalg->autopilot();

                    /* save 天线的相关 data */
                    fnameGPS = new QFile(filename);
                    fnameGPS->open(QIODevice::Append | QIODevice::Text);
                    QTextStream out(fnameGPS);
                    double sysTime = sysUsecTime();

                    out << "TIME:" << QString::number(sysTime, 'g', 10) << "\r\n";
                    out << "x:" << QString::number(data::lat, 'g', 12) << "\r\n";
                    out << "y:" <<QString::number(data::lng, 'g', 12) << "\r\n";
                    out << "theta:" <<data::theta << "\r\n";
                    out << "velocity:" << data::velocity << "\r\n";
                    out << "distance:" << data::distance <<"\r\n";
                    out << "calculatedfai:" << data::calculFai << "\r\n";

                    out << "AX:" << QString::number(data::AX_coor, 'g', 12) << "\r\n";
                    out << "AY:" << QString::number(data::AY_coor, 'g', 12) << "\r\n";
                    out << "BX:" << QString::number(data::BX_coor, 'g', 12) << "\r\n";
                    out << "BY:" << QString::number(data::BY_coor, 'g', 12) << "\r\n";

                    out << "north:"<<QString::number(data::north)<<"\r\n";
                    out <<"inValue"<<data::inValue<<"\r\n";

                    out << "x_mid:" << QString::number(data::x_mid,'g',12)<< "\r\n";
                    out << "y_mid:" << QString::number(data::y_mid,'g',12) <<"\r\n";

                    out << "AX_mid:" << QString::number(data::AX_coor_mid,'g',12) <<"\r\n";
                    out << "AY_mid:" << QString::number(data::AY_coor_mid,'g',12) <<"\r\n";
                    out << "BX_mid:" << QString::number(data::BX_coor_mid,'g',12) <<"\r\n";
                    out << "BY_mid:" << QString::number(data::BY_coor_mid,'g',12) <<"\r\n";
                    out << "distance_mid:" << data::distance_mid <<"\r\n";
                    out << "velValue:" << data::velValue <<"\r\n";
                    out << "dnorth:" << data::dnorth << "\r\n";
                    out << "Kp:" << data::Kp << "\r\n";
                    out << "Ki:" << data::Ki << "\r\n";
                    out << "Kd:" << data::Kd << "\r\n";
                    out << "chose:" << data::algorithmchose << "\r\n";
                    out << "rolldegree:" << data::rolldegree << "\r\n";
                    out << "diffTheta:"<<data::diffTheta <<"\r\n";
                    out << "expectedCircle:"<<data::expectedCircle <<"\r\n";
                    out << "slopeAB:" << data::slopeAB << "\r\n";
                    out << "xaMid_coor:" << QString::number(data::xaMid_coor,'g',12) << "\r\n";
                    out << "yaMid_coor:" <<  QString::number(data::yaMid_coor,'g',12)<< "\r\n";
                    out << "udiffTheta:" << data::udiffTheta << "\r\n";
                    out << "udistancesMid:" << data::udistanceMid << "\r\n";
                    out << "xInit:" << QString::number(data::xInit,'g',12) << "\r\n";
                    out << "yInit:" << QString::number(data::yInit,'g',12) << "\r\n";
                    out << "rollAngle:" << data::rollAngle << "\r\n";
                    out << "pitchAngle:" << data::pitchAngle << "\r\n";

                    fnameGPS->close();

                }

                else if(pilotflag == 2){
                    pilotalg->stop();
                    data::status = 99999;
                    pilotflag = 0;
                }

                if (data::status == 40103) {

                    /* send data to Bluetooth */
                    sprintf(blueSdata, "%s%lf,%lf,%d,%f,%f,%d,%f,%f,%f", "#", data::AX_coor_mid,data::AY_coor_mid,data::num,data::north,data::velocity,data::status,data::theta, data::distance_mid, data::calculFai);
                    strcat(blueSdata, "*");
                    strcat(blueSdata, "\n");
                    numBlue = serialBlue->Send(fBlue, blueSdata, strlen(blueSdata));
                    //                    printf("the send %s,and the length is %d\n", blueSdata,numBlue);
                }
                else if (data::status == 40104) {
                    /* send data to Bluetooth */
                    sprintf(blueSdata, "%s%lf,%lf,%d,%f,%f,%d,%f,%f,%f", "#", data::BX_coor_mid,data::BY_coor_mid,data::num,data::north,data::velocity,data::status,data::theta, data::distance_mid, data::calculFai);
                    strcat(blueSdata, "*");
                    strcat(blueSdata, "\n");
                    numBlue = serialBlue->Send(fBlue, blueSdata, strlen(blueSdata));
                    //                    printf("the send %s,and the length is %d\n", blueSdata,numBlue);
                }
                else {
                    /* send data to Bluetooth */
                    sprintf(blueSdata, "%s%lf,%lf,%d,%f,%f,%d,%f,%f,%f", "#", data::x_mid,data::y_mid,data::num,data::north,data::velocity,data::status,data::theta, data::distance_mid, data::calculFai);
                    strcat(blueSdata, "*");
                    strcat(blueSdata, "\n");
                    numBlue = serialBlue->Send(fBlue, blueSdata, strlen(blueSdata));
                    //                    printf("the send %s,and the length is %d\n", blueSdata,numBlue);
                }

            }
            else if((strstr(recv_buff, "HEADINGA") != NULL)) {

                data_handle(recv_buff);
            }
            else if((strstr(recv_buff, "GPVTG") != NULL)) {

                data_handle(recv_buff);
            }

        }
    }
    stopped=false;
}
/* 处理接受的天线数据，处理接受的命令数据
  */
void recvthread::data_handle(char data[])
{
    const char *sep = ",";    //define the split symbol
    int len=0;
    char *temp[30];
    char *pt;
    char gps_temp[16];

    pt = strtok(data, sep);

    while(pt) {
        temp[len] = pt;
        pt = strtok(NULL, sep);
        len++;
    }
    sprintf(gps_temp, "%s", temp[0]);
    /* 天线坐标、卫星数目等的数据 */
    if((strstr(gps_temp, "GPGGA") != NULL) && (len == 15)) {

        double x,y;

        sscanf(temp[2], "%lf", &x);
        x = x / 100.0;
        int lat_int = (int)x;
        x = lat_int + (x-lat_int)*100 / 60;

        sscanf(temp[4], "%lf", &y);
        y = y/100.0;
        int lng_int = (int)y;
        y= lng_int + (y - lng_int) * 100 /60;
        double *coordinate;
        coordinate = change(x, y);
        data::lat=coordinate[1];
        data::lng=coordinate[0];

        sscanf(temp[6], "%d", &data::num);
        sscanf(temp[1], "%lf", &data::GPStime);
    }
    /* 天线航向角 */
    else if((strstr(gps_temp, "HEADINGA") != NULL) && (len == 17)) {
        sscanf(temp[12], "%lf", &data::north);
        data::north = -data::north+360;
        sscanf(temp[13], "%lf", &data::rolldegree);
        data::rolldegree = -data::rolldegree;
    } else if((strstr(gps_temp, "GPVTG") != NULL) && ( len ==10 )) {
        sscanf(temp[7], "%lf", &data::velocity);
    }
    /* the blue receive data handle */
    else if (strstr(gps_temp, "#0") != NULL && len == 3) {
        sscanf(temp[1], "%d", &data::flagTemp);
    }

}
/* 坐标转换函数（高斯克吕格公式 */
double* recvthread::change(double B,double L)
{
    double L0 = 120;
    double a = 6378137;//a为椭球的长半轴:a=6378.137km
    double b = 6356752.3142;//b为椭球的短半轴:a=6356.7523141km
    //double e = sqrt(1 - pow(b, 2) / pow(a, 2));
    double e2 = 1 - pow(b, 2) / pow(a, 2);
    double ee = pow(a, 2) / pow(b, 2) - 1;
    double C1 =1.0 + 3.0 / 4.0 * e2 + 45.0 / 64.0 * pow(e2, 2) + 175.0 / 256.0 * pow(e2, 3) + 11025.0 / 16384.0 * pow(e2,4);
    double C2 = 3.0 / 4.0 * e2 + 15.0 / 16.0 * pow(e2, 2) + 525.0 / 512.0 * pow(e2, 3) + 2205.0 / 2048.0 * pow(e2, 4);
    double C3 = 15.0 / 64.0 * pow(e2, 2) + 105.0 / 256.0 * pow(e2, 3) + 2205.0 / 4096.0 * pow(e2, 4);
    double C4 = 35.0 / 512.0 * pow(e2, 3) + 315.0 / 2048.0 * pow(e2, 4);
    double C5 = 315.0 / 131072.0 * pow(e2, 4);
    double *x = (double *)malloc(16);

    B = B / 180 * M_PI;
    L = L / 180 * M_PI;
    L0 = L0 / 180 * M_PI;
    double l1 = L - L0;
    double t = tan(B);
    double n2 = ee*pow(cos(B), 2);
    double X = a*(1 - e2)*(C1*B - 1.0 / 2.0 * C2*sin(2 * B) + 1.0/ 4.0 * C3*sin(4 * B) - 1.0/6.0 * C4*sin(6 * B) + C5*sin(8 * B));
    double N = a / sqrt(1 - e2*pow(sin(B), 2));
    x[0]=X;
    x[0]+=(N*sin(B)*cos(B)*pow(l1,2)) / 2.0;
    x[0]+= (N*sin(B)*(pow(cos(B), 3))*(5.0 - pow(t, 2) + 9.0 * n2 + 4.0 * pow(n2, 2))*pow(l1, 4)) / 24.0;
    x[0]+=(N*sin(B)*(pow(cos(B), 5))*(61.0 - 58.0 * pow(t, 2) + pow(t, 4))*pow(l1, 6)) / 720.0;
    x[1] = N*cos(B)*l1;
    x[1]+= N*(pow(cos(B), 3)) * (1 - pow(t, 2) + n2)*pow(l1, 3) / 6;
    x[1]+= N*(pow(cos(B), 5)* (5 - 18 * t *t + pow(t, 4) + 14 * n2 - 58 * n2*(pow(t, 2)))*pow(l1, 5)) / 120;
    x[1]+=500000;

    return x;
}
/* 系统时间*/
double recvthread::sysUsecTime()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    double usecTime = tv.tv_sec + tv.tv_usec*pow(0.1,6);
    //    qDebug()<<tv.tv_sec;
    return usecTime;
}

void recvthread::stop(){

    /* shutdown the motor */
    pilotalg->stop();
    /* shutdown the serial */
    serialGPS->Close(fGPS);
    serial485->Close(f485);
    //    serialBlue->Close(fBlue);

    stopped=true;
}



