#ifndef TRAVERSALCONTROL_H
#define TRAVERSALCONTROL_H

#include<math.h>
#include<stdio.h>
#include<stdlib.h>
#include<data.h>

typedef struct PIDs
{
    double pfactor;
    double ifactor;
    double dfactor;
    double lasterror;
    double error;
    double sumerror;
	double error_2;
} PIDfactor;

typedef struct PPursuit {
    double Ld;
    double integratefactor;
} Ppursuitfactor;

typedef struct LikePD {
    double kp;
    double kd;
    double ki;
} LikePDfactor;

typedef struct NPD {
    double kp;
    double ki;
    double kd;
} nPD;

class traversalcontrol
{
public:
    traversalcontrol();
    double algorithmchose(int a, double carlength, double distance, double dnorth);
    void setPidFactor(int chose, double p, double i, double d);

private:
    double l;
    double d;
    double dnorth;
    double distanceintegrate;
    double pid();
    double purepursuit();
    double likepd();
    double disintegrate();
    double npd();

	double rbfpid();
	double caculateNorm2(double * tx,int count);
	double caculateHj(double* tx,double * tc,int count,double tbij);
	double caculateYpY(double * tx,double* ty,int count);
	void initPara();
	void updatePID(double yd,double ty);

	double kp_1,ki_1,kd_1;
	double inx[3];//输入向量x  
	double ci[6][3];//中心矢量C  
	double bi[6];//基宽向量B  
	double w[6];//权值向量W  
	double h[6];//径向基向量h  
	double ci_1[6][3],ci_2[6][3],ci_3[6][3];  
	double bi_1[6],bi_2[6],bi_3[6];  
	double w_1[6],w_2[6],w_3[6];  
	double u,u_1,y_1;  
	double xc[3];//增量式PID的输入
	//double outy,error;

    PIDfactor *pidfactor;
    Ppursuitfactor *ppursuitfactor;
    LikePDfactor *likepdfactor;
    nPD *npdfactor;
};

#endif // TRAVERSALCONTROL_H
