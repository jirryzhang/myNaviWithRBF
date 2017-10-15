/*
 * algorithm.h
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <QFile>
#include <QTextStream>
#include <qthread.h>
#include <QDateTime>
#include <QDir>
#include <QDebug>

/* 程序的头文件 */
#include "data.h"
#include "vel_fuzzycontrol.h"
#include "can00.h"  //控制转向电机的头文件
#include "can01.h"  //控制速度（电动推杆）的头文件
#include "turn.h"  //转弯算法的头文件
#include "traversalcontrol.h"

#define FAILIMIT (2*M_PI/7)	//FAILIMIT is the limit of front steering

class algorithm {

public:
	algorithm();
	virtual ~algorithm();

	void calcuteABline();       //计算AB线斜率的函数
    void calcuteP12line();      //计算地头直线斜率的函数
    double distanceP12();       //计算AB线斜率的函数
    double calculateD();        /* calculate the Distance */
	double calculateFai();      /* 计算期望的前轮转角值 */
    void autopilot();           /* 开始自动导航 */
    double  disintegrate();    //积分环节
    void stop();    //停止导航
    double pd();    //pd控制算法

	double l;       //前后轮轴距
	double Ld;      //前视距离
	double xa;      //A点x坐标
	double ya;      //A点y坐标
	double xb;      //B点x坐标
	double yb;      //B点y坐标
    double xa_mid;  //转换到后的A点x坐标
    double ya_mid;  //转换到后的A点y坐标
    double xb_mid;  //转换到后的B点x坐标
    double yb_mid;  //转换到后的B点y坐标
	double thetaAB; //AB线斜率
	double expected;//期望的前轮转角值
    double integrationfactor;//积分系数
    double integrate;

    /* 控制电机的变量 */
    struct can_frame frametemp;
    can00 *can;         //转向电机的对象
//    can00 *canMotor;    //电动推杆的对象

    /* 控制转弯的变量 */
    turn *turn1;
    double **dot;
    double circleFai();  //计算转弯时的期望前轮转角值

    int countAB;
    double thetaAB0;    //初始直线航向角
    double x0;
    double y0;
    double x1;
    double y1;
    double x2;
    double y2;

    /*地头直线的变量*/
    double px1;
    double py1;
    double px2;
    double py2;

    double thetaP12;
    unsigned int onTurn;       //转弯的判断标志
	
    vel_fuzzycontrol *velControl;  //行进速度的对象
    traversalcontrol *tracontrol;

};

#endif /* ALGORITHM_H_ */
