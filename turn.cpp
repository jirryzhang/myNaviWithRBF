#include "turn.h"

#include <iostream>
#include <fstream>

using namespace std;
turn::turn()
{
    //设定的一些值
    speed = 0.4;
    countnum = 0;
    rmin = 1.25;
    width = 2.5;
    turn_flag = 0;
    ang = 0;
    xs = 0;
    ys = 0;
    timestep = 0.1;

    /* 更新保存数据的文件 */
    int filenum = 0;
    filename = "data/" + QString::number(filenum) +"myfile.txt";
    fname = new QFile(filename);

    while (fname->exists()) {
        filenum++;
        filename = "data/" + QString::number(filenum) + "myfile.txt";
        fname = new QFile(filename);
    }
}

turn::~turn()
{
    //dtor
}
/* 转弯起始角，x坐标，y坐标 */
double** turn::genTurn(double angArg, double xsArg, double ysArg)
{
    ang = angArg;
    xs = xsArg;
    ys = ysArg;

	double** turnPath;   //表示标记点集合
	if(2*rmin>width){
		narrow=true;
		turnPath=generateTurnNarrow();  //调用genTurnNarrow()
	}
    else {
		narrow=false;
		turnPath=generateTurnWide(); // 调用genTurnWide()
	}
	return turnPath;
}

/* 行间距大于转弯半径 */
double **turn::generateTurnWide()
{
    double arc[3][5]={{0},{0},{0}};  //弓形线数组
	double x1,y1,x2,y2;

    if(data::turnFlag == 1)    //右转，圆弧-直线-圆弧
	{
		//第一段圆弧
		x1=xs+rmin*cos(ang-M_PI/2);  //圆心坐标计算
		y1=ys+rmin*sin(ang-M_PI/2);

		arc[0][0] = x1;
        arc[0][1] = y1;
        arc[0][2] = ang+M_PI/2;
        arc[0][3] = M_PI/2;
        arc[0][4] = 0;

		//末端圆弧
		x2=xs+(width-rmin)*cos(ang-M_PI/2);   //圆心坐标
		y2=ys+(width-rmin)*sin(ang-M_PI/2);

		arc[2][0] = x2;
        arc[2][1] = y2;
        arc[2][2] = ang;
        arc[2][3] = M_PI/2;
        arc[2][4] = 0;

		double line_x1 = x1+rmin*cos(ang);
		double line_y1 = y1+rmin*sin(ang);
		double line_x2 = x2+rmin*cos(ang);
		double line_y2 = y2+rmin*sin(ang);

		arc[1][0] = line_x1;
        arc[1][1] = line_y1;
        arc[1][2] = line_x2;
        arc[1][3] = line_y2;
        arc[1][4] = 0;

        /* 测试三段弧线 */
        for(int i=0;i<3;i++)
        {
            printf("the x = %f\n",arc[i][0]);
            printf("the y = %f\n",arc[i][1]);
            printf("the startAng = %f\n",arc[i][2]);
            printf("the theta = %f\n",arc[i][3]);
            printf("the  = %f\n",arc[i][4]);
            printf("\n");
        }
	}
    else if(data::turnFlag == 2)   //左转，过程同上
	{
		x1=xs+rmin*cos(ang+M_PI/2);
		y1=ys+rmin*sin(ang+M_PI/2);
		arc[0][0] = x1;
        arc[0][1] = y1;
        arc[0][2] = ang-M_PI/2;
        arc[0][3] = M_PI/2;
        arc[0][4] = 1;

		x2=xs+(width-rmin)*cos(ang+M_PI/2);
		y2=ys+(width-rmin)*sin(ang+M_PI/2);
		arc[2][0] = x2;
        arc[2][1] = y2;
        arc[2][2] = ang;
        arc[2][3] = M_PI/2;
        arc[2][4] = 1;

		double line_x1 = x1+rmin*cos(ang);
		double line_y1 = y1+rmin*sin(ang);
		double line_x2 = x2+rmin*cos(ang);
		double line_y2 = y2+rmin*sin(ang);

		arc[1][0] = line_x1;
        arc[1][1] = line_y1;
        arc[1][2] = line_x2;
        arc[1][3] = line_y2;
        arc[1][4] = 0;
	}
    /* 测试三段弧线 */
    for(int i=0;i<3;i++) {
		
        printf("the x = %f\n",arc[i][0]);
        printf("the y = %f\n",arc[i][1]);
        printf("the startAng = %f\n",arc[i][2]);
        printf("the theta = %f\n",arc[i][3]);
        printf("the  = %f\n",arc[i][4]);
        printf("\n");
    }

	return segmentation(arc);
}
/* 行距小于转弯半径 */
double **turn::generateTurnNarrow()
{
    double arc[3][5]={{0},{0},{0}};
//    double *arc[3];

    double turnAng, startAng1, x1, y1, x2, y2, startAng2, turnAng2, startAng3, x3, y3;
    if(data::turnFlag == 1) {
        turnAng = acos((rmin +width/2)/2/rmin);
        startAng1 = ang - M_PI/2;
        x1 = xs - rmin*cos(startAng1);
        y1 = ys - rmin*sin(startAng1);

        arc[0][0] = x1;
        arc[0][1] = y1;
        arc[0][2] = startAng1;
        arc[0][3] = turnAng;
        arc[0][4] = 1;

        //第二段圆弧
        turnAng2 = M_PI + 2*turnAng;  //圆弧角
        x2 = xs + width/2*cos(startAng1) +2 *rmin*sin(turnAng)*cos(ang);  //圆心坐标
        y2 = ys + width/2*sin(startAng1) + 2*rmin*sin(turnAng)*sin(ang);
        startAng2 = startAng1 + M_PI + turnAng;   //起始角

//        double temp_arc1[5] = {x2 ,y2, startAng2, turnAng2, 0};
//        arc[1] = temp_arc1;
        arc[1][0] = x2;
        arc[1][1] = y2;
        arc[1][2] = startAng2;
        arc[1][3] = turnAng2;
        arc[1][4] = 0;

        startAng3 = ang + M_PI/2 - turnAng;
        x3=xs+(width+rmin)*cos(startAng1);
        y3=ys+(width+rmin)*sin(startAng1);

//        double temp_arc2[5] = {x3 ,y3, startAng3, turnAng, 1};
//        arc[2] = temp_arc2;
        arc[2][0] = x3;
        arc[2][1] = y3;
        arc[2][2] = startAng3;
        arc[2][3] = turnAng;
        arc[2][4] = 1;
    }
    else if(data::turnFlag == 2) //左转，计算过程同上
	{
		turnAng=acos((rmin+width/2)/(2*rmin));
        startAng1=ang+M_PI/2;
        x1=xs+rmin*cos(ang-M_PI/2);
        y1=ys+rmin*sin(ang-M_PI/2);

        arc[0][0] = x1;
        arc[0][1] = y1;
        arc[0][2] = startAng1;
        arc[0][3] = turnAng;
        arc[0][4] = 0;

        turnAng2=M_PI+2*turnAng;
        x2=xs+width/2*cos(ang+M_PI/2)+2*rmin*sin(turnAng)*cos(ang);
        y2=ys+width/2*sin(ang+M_PI/2)+2*rmin*sin(turnAng)*sin(ang);
        startAng2=startAng1-M_PI - turnAng;

        arc[1][0] = x2;
        arc[1][1] = y2;
        arc[1][2] = startAng2;
        arc[1][3] = turnAng2;
        arc[1][4] = 1;

        startAng3= ang - M_PI/2 + turnAng;
        x3=xs+(width+rmin)*cos(startAng1);
        y3=ys+(width+rmin)*sin(startAng1);

        arc[2][0] = x3;
        arc[2][1] = y3;
        arc[2][2] = startAng3;
        arc[2][3] = turnAng;
        arc[2][4] = 0;
	}
	/* 测试圆弧圆心所在 */
    for(int i=0;i<3;i++) {
        printf("the x = %f\n",arc[i][0]);
        printf("the y = %f\n",arc[i][1]);
        printf("the startAng = %f\n",arc[i][2]);
        printf("the theta = %f\n",arc[i][3]);
        printf("the  = %f\n",arc[i][4]);
        printf("\n");
    }

	return segmentation(arc);
}
/* 圆弧分成相应的点 */
double **turn::segmentation(double arg[3][5])
{
    countnum=0;

    double (*threeArc)[5];
    threeArc = arg;

	double **waypts;
   /* C语言动态分配内存 */
//	waypts = (double **)malloc(10*sizeof(double *));
    /* C++动态分配内存 */
    waypts = new double* [3200];

	//当前标记点坐标、方向角、圆心坐标、圆心角、终止圆弧角、时间、是否逆时针
    double xt,yt,dat,xc,yc,startAng,endAng,t,aclk;

	/* 三段圆弧的路径规划点 */
	if(narrow) {

        for(int i=0; i<3; i++) {

            double *arc = new double[5];             //当前所在圆弧段

            arc = threeArc[i];
            t=0;                                      //从3段圆弧数组中获取各参数
            startAng = arc[2];                       //起始圆弧角
            endAng = startAng  + arc[3];                             //终止角=起始角+圆弧角
            xc=arc[0];                                //圆心坐标与逆顺时针
            yc=arc[1];
            aclk=arc[4];
            while(fabs(startAng-arc[2]) <= fabs(arc[3]))        //角度未超过终止角，在圆弧上
            {
                /* C语言动态分配内存 */
            //        waypts[countnum] = (double *)malloc(4*sizeof(double));
                /* C++ 动态分配内存 */
                waypts[countnum] = new double[4];
                xt=xc+rmin*cos(startAng);                      //当前位置
                yt=yc+rmin*sin(startAng);
                if(aclk) {
                    dat=startAng+M_PI/2;                          //求行驶方向角
                    startAng += speed*timestep/rmin;                //角度按照时间步长步进
                }
                else {
                    dat = startAng - M_PI/2;                          //求行驶方向角
                    startAng -= speed*timestep/rmin;                //角度按照时间步长步进
                }

            //    /* 这样得不到想要的结果*/
            //        double waypt[4]={xt,yt,dat,t};            //标记点（x、y坐标、行驶方向、时间）
            //        waypts[countnum]=waypt;                     //记录在路径点中

                waypts[countnum][0] = xt;
                waypts[countnum][1] = yt;
                waypts[countnum][2] = dat;
                waypts[countnum][3] = t;                     //记录在路径点中

                countnum++;
                t+=timestep;

            }
            printf("the countnum = %d", countnum);
        }
	}
	/* 圆弧直线圆弧的路径点 */
	else {
        for(int i=0; i<3; i++) {
            double *arc = new double[5];
            arc = threeArc[i];

            t=0;
            /* 直线的点 */
            if(i == 1) {
                double x1=arc[0],y1=arc[1],x2=arc[2],y2=arc[3];  //获取起止点坐标

                dat=atan2(y2-y1,x2-x1);   //直线方向角
                xt=x1;
                yt=y1;
                while((fabs(xt-x1) <= fabs(x2-x1)) && (fabs(yt-y1) <= fabs(y2-y1)))   //x坐标在起止点之间，在直线上
                {
                    waypts[countnum] = new double[4];
//                    printf("%f\n", fabs(1.2222));
                    waypts[countnum][0] = xt;
                    waypts[countnum][1] = yt;
                    waypts[countnum][2] = dat;
                    waypts[countnum][3] = t;                     //记录在路径点中

                    double dis = speed*timestep;     //距离增加一个步长
                    xt += dis*cos(dat);             //标记点x,y坐标更新
                    yt += dis*sin(dat);

                    countnum++;
                    t+=timestep;
                }
                printf("the countnum = %d", countnum);
            }
            /* 圆弧的点 */
            else {

                startAng=arc[2];                       //起始圆弧角
                endAng = startAng + arc[3];                             //终止角=起始角+圆弧角
                xc=arc[0];                                //圆心坐标与逆顺时针
                yc=arc[1];
                aclk=arc[4];

                while(fabs(startAng-arc[2]) <= fabs(arc[3]))        //角度未超过终止角，在圆弧上
                {
                    /* C语言动态分配内存 */
                //        waypts[countnum] = (double *)malloc(4*sizeof(double));
                    /* C++ 动态分配内存 */
                    waypts[countnum] = new double[4];
                    xt = xc+rmin*cos(startAng);                      //当前位置
                    yt = yc+rmin*sin(startAng);
                    if(aclk) {
                        dat = startAng+M_PI/2;                          //求行驶方向角
                        startAng += speed*timestep/rmin;                //角度按照时间步长步进
                    }
                    else {
                        dat = startAng - M_PI/2;                          //求行驶方向角
                        startAng -= speed*timestep/rmin;                //角度按照时间步长步进
                    }

                //    /* 这样得不到想要的结果*/
                //        double waypt[4]={xt,yt,dat,t};            //标记点（x、y坐标、行驶方向、时间）
                //        waypts[countnum]=waypt;                     //记录在路径点中

                    waypts[countnum][0] = xt;
                    waypts[countnum][1] = yt;
                    waypts[countnum][2] = dat;
                    waypts[countnum][3] = t;                     //记录在路径点中

                    countnum++;
                    t+=timestep;
//                    printf("the countnum = %d", countnum);
                }
                printf("the countnum = %d", countnum);
            }
        }
    }

    /* QT保存文件 */
    fname = new QFile(filename);
    fname->open(QIODevice::Append | QIODevice::Text);
    QTextStream out(fname);

    for(int i=0; i< countnum; i++) {
        out<<"x:"<<QString::number(waypts[i][0],'g',12)<<"\r\n";
        out<<"y:"<<QString::number(waypts[i][1],'g',12)<<"\r\n";
    }
    out<<"countnum" <<countnum<<"\r\n";
    fname->close();

    printf("%d\n", countnum);
	return waypts;   //返回标记点集合
}
