/*
 * algorithm.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#include "algorithm.h"
#include "data.h"

#define LD0 2
#define IF_INTEGRATE 1
#define DISTANCEINITIAL 0
#define CHOSE 2
algorithm::algorithm() {

    can=new can00();
//    canMotor = new can00();

    l=1.06;
    Ld=LD0;
    thetaAB=0;
    expected=0;
    integrate = 0;
    integrationfactor=0.08;

    xa=0,xb=0,ya=0,yb=0;
    xa_mid=0,xb_mid=0,ya_mid=0,yb_mid=0;

    velControl = new vel_fuzzycontrol();
    tracontrol = new traversalcontrol();
    /* 转弯变量的初始化 */
    turn1 = new turn();

    countAB = 0;
    thetaAB0 = 0;              //原AB线航向角初始值为零
    x0 = 0;
    y0 = 0;
    x1 = 0;
    y1 = 0;
    x2 = 0;
    y2 = 0;
    px1 = 0, py1 = 0;
    px2 = 0 ,py2 = 0;

    thetaP12 = 0;
    onTurn = 2;
    /* 角度传感器初始值校准　*/
    data::initTheta = 79.10;

    /*选择预运行算法*/
    data::algorithmchose = CHOSE;
    /*设置算法参数*/
    data::Kp = 1;
    data::Ki = 0;
    data::Kd = 3.5;
    tracontrol->setPidFactor(data::algorithmchose, data::Kp, data::Ki, data::Kd);

}

algorithm::~algorithm() {
    // TODO Auto-generated destructor stub
}

void algorithm::autopilot() {

    /* 直线导航*/
    if (data::turnFlag == 0 && data::num == 4) {
        data::udiffTheta = 0;
        data::udistanceMid = 0;
        data::expectedCircle = 0;

        if (xa_mid == xb_mid && xa_mid == 0) {
            this->calcuteABline();
        }
        /* 低头转弯的直线设定　*/
        if (px1 == px2 && px1 == 0) {
            this->calcuteP12line();
        }
        /* 地头转弯的距离设定　*/
        double farmDis = distanceP12();
        printf("the farmdis = %f\n", fabs(farmDis));

        if(fabs(farmDis) > 60 && onTurn == 1) {
            data::turnFlag = 1;
            onTurn = 2;//左转判断指令
        } else if (fabs(farmDis) <10 && onTurn == 2) {
            data::turnFlag = 2;
            onTurn = 1; // 右转判断指令
        } else {
            data::turnFlag = 0;
        }

        data::status = 10010;   //直线返回的状态指令

        //计算横向偏差
        data::distance_mid = calculateD();

        //计算航向角偏差,范围应该控制在(-180,180)
        data::diffTheta = data::north - thetaAB/M_PI*180;
        while(data::diffTheta < -180)
            data::diffTheta += 360;
        while (data::diffTheta > 180)
            data::diffTheta -= 360;
        printf("difftheta:%lf\n",data::diffTheta);
        data::dnorth = data::diffTheta;

        /*
        //void traversalcontrol::algorithmchose(int a, double carlength, double distance, double dnorth)
        //a = 0时为PID控制
        //a = 1时为纯追踪模型
        //a = 2时为类PD控制
        //carlength 表示前后轴距
        //distance 表示横向偏差
        //dnorth 表示航向角偏差，单位为弧度
        */
        data::calculFai = tracontrol->algorithmchose(CHOSE, this->l, data::distance_mid, data::dnorth/180*M_PI);
        /* 期望转角值 */
        expected = data::calculFai*180/M_PI;
        double comp = (-1.7*(data::theta+data::initTheta)) + expected;

        /* 20 is jiansubi, time is 0.1s */
        int a = 0;
        a = (int)(comp*16*100/36)*37.5;
        data::motor_vel=a;
        qDebug() << "a=" << a;
        /* 转向电机控制 */
        if(a>20000) {
            can->can_send(20000);
        } else if( a<-20000) {
            can->can_send(-20000);
        } else if ( (a<200 ) && (a>-200) ) {
            can->can_send(0);
        } else {
            can->can_send(a);
        }

        /* 前进速度控制 */
        //航向角偏差

        data::velValue = velControl->fuzzyOutput(data::distance_mid, data::diffTheta);
        //        printf("the velocity=%f\n", data::velValue);
        int velPosition = (int)(data::velValue*28);

        if( velPosition > 28) {
            velPosition = 28;
        } else if(velPosition <= 0) {
            velPosition = 0;
        }
//        canMotor->velocityCan(velPosition);
        qDebug()<<"vel="<<velPosition;
        can->velocityCan(velPosition);

    }
    /* 拐弯的命令都设定为右拐 */
    else if(data::turnFlag == 1 && data::num == 4) {

//        canMotor->velocityCan(14);  //先减速再转弯
//        if(canMotor->can_recv(&frametemp) > 0 && (int)frametemp.data[0] < 16) {
        can->velocityCan(14);  //先减速再转弯
        if(can->can_recv(&frametemp) > 0 && (int)frametemp.data[0] < 16) {

            //            double expectedCircle = circleFai()*180/M_PI;   //转弯算法的期望转角
            /* 设定追踪的下一条直线　*/
            if(data::ABturnFlag == 1 && data::status != 10020) {
                xa_mid = xa_mid + 2.2*cos(thetaAB-M_PI/2);
                ya_mid = ya_mid + 2.2*sin(thetaAB-M_PI/2);
                if(thetaAB > M_PI) {
                    thetaAB = thetaAB - M_PI;
                } else{
                    thetaAB = thetaAB + M_PI;
                }
                data::ABturnFlag = 0;
                data::slopeAB = thetaAB;
                data::xaMid_coor = xa_mid;
                data::yaMid_coor = ya_mid;
            }
            data::status = 10020;   //右转返回的状态指令

            data::expectedCircle = -34;
            double comp = -1*(data::theta+data::initTheta) + data::expectedCircle;

            /* 航偏角　*/
            data::diffTheta = fabs(data::north - thetaAB/M_PI*180);
            if(data::diffTheta > 180)
                data::diffTheta -=180;

            /* 20 is jiansubi, time is 0.1s */
            int a = 0;
            a = (int)(comp*16*100/36)*37.5;

            if(a>20000) {
                can->can_send(20000);
            } else if( a<-20000) {
                can->can_send(-20000);
            } else {
                can->can_send(a);
            }

            /* 检测到下一个直线的位置 */

            data::distance_mid =calculateD();
            printf("the distance_mid:%f\n",data::distance_mid);
            printf("the difftheta:%f.\n",data::diffTheta);
            /* 转弯的阈值设置 */
            if(fabs(data::distance_mid)<0.3 && fabs(data::diffTheta)<40) {
                data::turnFlag = 0;
                data::udiffTheta = data::diffTheta;
                data::udistanceMid = data::distance_mid;
            }
        } else {
            /* 如果速度没有降低下来，则继续直线导航　*/
            //计算横向偏差
            data::distance_mid = calculateD();

            //计算航向角偏差,范围应该控制在(-180,180)
            data::diffTheta = data::north - thetaAB/M_PI*180;
            while(data::diffTheta < -180)
                data::diffTheta += 360;
            while (data::diffTheta > 180)
                data::diffTheta -= 360;
            printf("difftheta:%lf\n",data::diffTheta);
            data::dnorth = data::diffTheta;

            data::calculFai = tracontrol->algorithmchose(CHOSE, this->l, data::distance_mid, data::dnorth/180*M_PI);
            /* 期望转角值 */
            expected = data::calculFai*180/M_PI;
            double comp = (-1.7*(data::theta+data::initTheta)) + expected;

            /* 20 is jiansubi, time is 0.1s */
            int a = 0;
            a = (int)(comp*16*100/36)*37.5;
            data::motor_vel=a;
            qDebug() << "a=" << a;
            /* 转向电机控制 */
            if(a>20000) {
                can->can_send(20000);
            } else if( a<-20000) {
                can->can_send(-20000);
            } else if ( (a<200 ) && (a>-200) ) {
                can->can_send(0);
            } else {
                can->can_send(a);
            }
            data::status = 10011; //转弯段的减速过程的直线
        }
    }
    /* 左转控制 */
    else if (data::turnFlag == 2 && data::num == 4) {
        /* 降低速度 */
//        canMotor->velocityCan(14);
//        /* 设定追踪的下一条直线　*/
//        if(canMotor->can_recv(&frametemp) && (int)frametemp.data[0] < 16) {
        can->velocityCan(14);  //先减速再转弯
        if(can->can_recv(&frametemp) > 0 && (int)frametemp.data[0] < 16) {

            //            double expectedCircle = circleFai()*180/M_PI;
            if(data::ABturnFlag == 2 && data::status != 10030) {

                xa_mid = xa_mid - 2.2*cos(thetaAB-M_PI/2);
                ya_mid = ya_mid - 2.2*sin(thetaAB-M_PI/2);

                if(thetaAB > M_PI) {
                    thetaAB = thetaAB - M_PI;
                } else{
                    thetaAB = thetaAB + M_PI;
                }
                data::ABturnFlag = 0;
                data::slopeAB    = thetaAB;  //保存斜率
                data::xaMid_coor = xa_mid;
                data::yaMid_coor = ya_mid;
            }

            data::status = 10030;   //左转返回的状态指令
            data::expectedCircle = 33;
            double comp = -1*(data::theta+data::initTheta) + data::expectedCircle;
            /* 航偏角　*/
            data::diffTheta = fabs(data::north - thetaAB/M_PI*180);
            if(data::diffTheta > 180)
                data::diffTheta -=180;
            /* 20 is jiansubi, time is 0.1s */
            int a = 0;
            a = (int)(comp*16*100/36)*37.5;

            if(a>20000) {
                can->can_send(20000);
            } else if( a<-20000) {
                can->can_send(-20000);
            } else {
                can->can_send(a);
            }
            /* 检测距离下一个直线的长度 */
            data::distance_mid = calculateD();
            printf("the distance_mid:%f\n",data::distance_mid);
            printf("the difftheta:%f.\n",data::diffTheta);

            if(fabs(data::distance_mid)<0.3 && fabs(data::diffTheta)<40) {
                data::turnFlag = 0;
                data::udiffTheta = data::diffTheta;
                data::udistanceMid = data::distance_mid;
            }
        } else{
            /* 如果速度没有降低下来，则继续直线导航　*/
            //计算横向偏差
            data::distance_mid = calculateD();

            //计算航向角偏差,范围应该控制在(-180,180)
            data::diffTheta = data::north - thetaAB/M_PI*180;
            while(data::diffTheta < -180)
                data::diffTheta += 360;
            while (data::diffTheta > 180)
                data::diffTheta -= 360;
            printf("difftheta:%lf\n",data::diffTheta);
            data::dnorth = data::diffTheta;

            data::calculFai = tracontrol->algorithmchose(CHOSE, this->l, data::distance_mid, data::dnorth/180*M_PI);
            /* 期望转角值 */
            expected = data::calculFai*180/M_PI;
            double comp = (-1.7*(data::theta+data::initTheta)) + expected;

            /* 20 is jiansubi, time is 0.1s */
            int a = 0;
            a = (int)(comp*16*100/36)*37.5;
            data::motor_vel=a;
            qDebug() << "a=" << a;
            /* 转向电机控制 */
            if(a>20000) {
                can->can_send(20000);
            } else if( a<-20000) {
                can->can_send(-20000);
            } else if ( (a<200 ) && (a>-200) ) {
                can->can_send(0);
            } else {
                can->can_send(a);
            }

            data::status = 10011; //转弯段的减速过程的直线
        }

    } else {
        /* 导航状态标志位不正确时停止导航 */
//        canMotor->velocityCan(0);
        can->velocityCan(0);
        can->can_send(99999);
    }
}

/* calculate the ABline */
void algorithm::calcuteABline() {
    //the location of A,B is A(xa,ya) and B(xb,yb)

    QFile *fABname = new QFile("/opt/GPS.txt");
    fABname->open(QIODevice::ReadWrite | QIODevice::Text);
    QTextStream out(fABname);
    xa = out.readLine().toDouble();
    ya = out.readLine().toDouble();
    xb = out.readLine().toDouble();
    yb = out.readLine().toDouble();
    xa_mid = out.readLine().toDouble();
    ya_mid = out.readLine().toDouble();
    xb_mid = out.readLine().toDouble();
    yb_mid = out.readLine().toDouble();
    fABname->close();
    //    xa_mid = data::AX_coor_mid;
    //    ya_mid = data::AY_coor_mid;
    //    xb_mid = data::BX_coor_mid;
    //    yb_mid = data::BY_coor_mid;
    /* C++中的函数 */
    thetaAB = atan2(yb_mid-ya_mid,xb_mid-xa_mid);

    if(thetaAB<0) {
        thetaAB += 2*M_PI;
    }
    data::xaMid_coor = xa_mid;
    data::yaMid_coor = ya_mid;
    data::slopeAB = thetaAB;
    //    qDebug()<<"thetaAB = "<<thetaAB;
}
/* 计算地头直线斜率的函数 */
void algorithm::calcuteP12line()
{
    QFile *farm = new QFile("/opt/farm.txt");
    farm->open(QIODevice::ReadWrite | QIODevice::Text);
    QTextStream out(farm);
    px1 = out.readLine().toDouble();
    py1 = out.readLine().toDouble();
    px2 = out.readLine().toDouble();
    py2 = out.readLine().toDouble();

    farm->close();

    thetaP12 = atan2(py2-py1,px2-px1);
    if(thetaP12<0) {
        thetaP12 += 2*M_PI;
    }

}
/* 计算地头的距离
 * 有正负 */
double algorithm::distanceP12()
{
    double x=data::x_mid;
    double y=data::y_mid;
    double d=0;

    double k = tan(thetaP12);
    double b = py1-k*px1;

    if(thetaP12<M_PI/2 || thetaP12 >M_PI*3/2) {
        d = (k*x + b- y)/sqrt(1+k*k);
    } else if(M_PI/2<thetaP12 && thetaP12 < M_PI*3/2) {
        d = -(k*x + b- y)/sqrt(1 + k*k);
    } else if( thetaP12 == (M_PI/2) ){
        d=x-px1;
    } else if( thetaP12 == (3*M_PI/2) ) {
        d=px1-x;
    }

    return d;
}

/* calculate the Distance */
double algorithm::calculateD(){

    double x=data::x_mid;
    double y=data::y_mid;
    double d=0;

    double k = tan(thetaAB);
    double b = ya_mid-k*xa_mid;

    if(thetaAB<M_PI/2 || thetaAB >M_PI*3/2) {
        d = (k*x + b- y)/sqrt(1+k*k) + DISTANCEINITIAL;
    } else if(M_PI/2<thetaAB && thetaAB < M_PI*3/2) {
        d = -(k*x + b- y)/sqrt(1 + k*k) + DISTANCEINITIAL;
    } else if( thetaAB == (M_PI/2) ){
        d=x-xa_mid + DISTANCEINITIAL;
    } else if( thetaAB == (3*M_PI/2) ) {
        d=xa_mid-x + DISTANCEINITIAL;
    }

    data::distance_mid =d;

    data::distance = 0;
    return d;
}

//计算转弯时的期望前轮转角值 */
double algorithm::circleFai()
{
    if (turn1->countnum == 0 ) {
        double angArg = thetaAB;     //暂以航向角顺时针90度为拖拉机行进的方向
        thetaAB0 = thetaAB;
        printf("the angArg= %f\n", angArg);

        //        double xsArg = data::lat;
        //        double ysArg = data::lng;
        //AB线投影点
        double k = tan(thetaAB);
        double b = ya_mid-k*xa_mid;

        double xsArg = (data::x_mid+ k*(data::y_mid- b))/(1+ k*k);
        double ysArg = (data::x_mid*k+ k*k*data::y_mid + b)/(1+ k*k);

        dot = turn1->genTurn(angArg, xsArg, ysArg);
    }
    double x = data::x_mid;
    double y = data::y_mid;

    double circleThetaAB = 0;

    /* 计算斜率 */
    if(countAB < (turn1->countnum-2)) {

        x0 = dot[countAB][0];
        y0 = dot[countAB][1];
        x1 = dot[countAB+1][0];
        y1 = dot[countAB+1][1];
        x2 = dot[countAB+2][0];
        y2 = dot[countAB+2][1];

        double vectorAB = 0;
        vectorAB = (x1-x0)*(x - x1) + (y1- y0)*(y-y1);

        if( vectorAB >= 0) {

            circleThetaAB = atan2(y2-y1, x2-x1);
            if(circleThetaAB < 0) {
                circleThetaAB += 2*M_PI;
            }
            countAB++;
        } else {
            //C++中的函数
            circleThetaAB = atan2(y1-y0,x1-x0);
            if(circleThetaAB < 0) {
                circleThetaAB += 2*M_PI;
            }
        }

        printf("the countAB = %d\n" , countAB);
        printf("the turnflag%d \n", data::turnFlag);
    }else {
        /* 改变AB线的斜率（反向）*/
        if(thetaAB > M_PI ) {
            data::turnFlag = 0;
            turn1->countnum = 0;   //拐弯计数点个数归零
            //计算下一次拐弯的点
            countAB = 0;
            xa_mid = x2;           //下一段AB线的起点x坐标
            ya_mid = y2;           //下一段AB线的起点x坐标
            thetaAB = thetaAB - M_PI;
            circleThetaAB = thetaAB;
            integrate = 0;
            data::slopeAB = thetaAB;
            data::xaMid_coor = xa_mid;
            data::yaMid_coor = ya_mid;

        }else {
            data::turnFlag = 0;
            turn1->countnum = 0;
            countAB = 0;
            xa_mid = x2;
            ya_mid = y2;
            thetaAB = thetaAB + M_PI;
            circleThetaAB = thetaAB;
            integrate = 0;
            data::slopeAB = thetaAB;
            data::xaMid_coor = xa_mid;
            data::yaMid_coor = ya_mid;
        }
        //        printf("the thetaAB0 = %f\n", thetaAB);
    }

    /* 计算横向偏差 */
    double d=0;
    double k = tan(circleThetaAB);
    double b = y1-k*x1;

    if(circleThetaAB<M_PI/2 || circleThetaAB >M_PI*3/2) {
        d = (k*x + b- y)/sqrt(1+k*k);
    }else if(M_PI/2<circleThetaAB && circleThetaAB < M_PI*3/2) {
        d = -(k*x + b- y)/sqrt(1 + k*k);
    }else if( circleThetaAB == (M_PI/2) ){
        d=x-x1;
    }else if( circleThetaAB == (3*M_PI/2) ) {
        d=x1-x;
    }

    data::distance_mid = d;
    data::distance = 0;

    if(fabs(d)>fabs(Ld)){
        Ld = 1.8 *fabs(d);
    }
    else {
        Ld = 1.8;
    }

    /* 计算前轮转角 */
    double northTheta = data::north;
    northTheta = northTheta / 180*M_PI;
    double fai = 0;
    /* 转弯的纯追踪算法 */
    fai = atan(-2*l*(-d*cos(northTheta-circleThetaAB) + sqrt(Ld*Ld - d*d)*sin(northTheta-circleThetaAB))/Ld/Ld);
    data::calculFai = fai;

    if(fai>FAILIMIT) {
        fai = FAILIMIT;
    }
    else if(fai<-FAILIMIT) {
        fai = -FAILIMIT;
    }

    return fai;
}

/* stop the motor */
void algorithm::stop() {
//    canMotor->velocityCan(0);  //速度电机回到原始位置
    can->velocityCan(0);
    can->can_send(99999);	   //转向电机控制脱机指令
}


