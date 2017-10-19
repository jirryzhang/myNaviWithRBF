#include "traversalcontrol.h"
#define IF_INTEGRATE 1
#define FAILIMIT M_PI/4
#define LD0 2


#include <iostream>  
#include <math.h>  

using namespace std; 
double eta=0.5;//学习率  
double alfa=0.05;//动量因子
double etaP=0.15,etaI=0.15,etaD=0.15;//P,I,D系数调整速度  


traversalcontrol::traversalcontrol()
{
    l = 0;
    d = 0;
    dnorth = 0;
    distanceintegrate = 0;

    pidfactor = (PIDfactor*)calloc(1,sizeof(PIDfactor));
    pidfactor->pfactor = 0.2;
    pidfactor->ifactor = 0.01;
    pidfactor->dfactor = 1.2;
    pidfactor->error = 0;
    pidfactor->lasterror = 0;
    pidfactor->sumerror = 0;

    ppursuitfactor = (Ppursuitfactor*)calloc(1,sizeof(Ppursuitfactor));
    ppursuitfactor->Ld = LD0;

    likepdfactor = (LikePDfactor*)calloc(1,sizeof(LikePDfactor));
    likepdfactor->kp = 0.255;
    likepdfactor->kd = 1;
    likepdfactor->ki = 0;

    npdfactor = (nPD*)calloc(1,sizeof(nPD));
    npdfactor->kp = 1;
    npdfactor->ki = 0;
    npdfactor->kd = 3;

	initPara();
}

double traversalcontrol::algorithmchose(int a, double carlength, double distance, double dnorth) {
    this->l = carlength;
    this->d = distance;
    this->dnorth = dnorth;
    if (fabs(this->dnorth) > 3.14) {
        printf("Please check the unit of degree!");
    }
    switch(a) {
    case 0:
        return pid();
        break;
    case 1:
        return purepursuit();
        break;
    case 2:
        return likepd();
        break;
    case 3:
        return npd();
        break;
    default:
        return purepursuit();
        break;
    }
}

double traversalcontrol::pid() {
    pidfactor->error = this->d;
    pidfactor->sumerror += this->d;
    double fai = 0;
    //横线偏差相见做差分
    //fai = pidfactor->pfactor*pidfactor->error + pidfactor->ifactor*pidfactor->sumerror + pidfactor->dfactor*(pidfactor->error-pidfactor->lasterror);
    //航向角作为查分
    fai = pidfactor->pfactor*pidfactor->error + pidfactor->ifactor*pidfactor->sumerror + pidfactor->dfactor*(data::dnorth/180*M_PI);
    pidfactor->lasterror = pidfactor->error;
    return fai;
}

double traversalcontrol::purepursuit() {
    if((fabs(this->d)) >= ppursuitfactor->Ld){
        ppursuitfactor->Ld = 3*fabs(this->d);
    }
    else{
        ppursuitfactor->Ld = LD0;  // the initial value
    }

    //data::Ld = fuzzyCompute->fuzzyOutput(d, data::velocity/3.6);
    double fai=0;

    fai = atan(-2*this->l*(-this->d*cos(this->dnorth) + sqrt((ppursuitfactor->Ld)*(ppursuitfactor->Ld) - this->d*this->d)*sin(this->dnorth))/ppursuitfactor->Ld/ppursuitfactor->Ld)+ this->disintegrate()*ppursuitfactor->integratefactor*IF_INTEGRATE;
    if ((fai<=FAILIMIT)&&(fai>=-FAILIMIT))
    {
        fai=fai;
    }
     if(fai>FAILIMIT) {
        fai=FAILIMIT;
    }
    if(fai<-FAILIMIT) {
        fai=-FAILIMIT;
    }
    return fai;
}

double traversalcontrol::likepd() {
    double fai = atan(this->l*cos(this->dnorth)*cos(this->dnorth)*cos(this->dnorth)*(-likepdfactor->kd*tan(this->dnorth)+likepdfactor->kp*this->d)) + this->disintegrate()*likepdfactor->ki*IF_INTEGRATE;
    return fai;
}

double traversalcontrol::npd() {
    double fai = -npdfactor->kd * this->dnorth + npdfactor->kp * pow(fabs(this->d),-0.5) * this->d;
    return fai;
}

double traversalcontrol::disintegrate() {
    if(fabs(data::distance_mid)<0.1) {
        if (this->distanceintegrate > 100) {
            if(this->d > 0)
            {
                this->distanceintegrate += 0;
            }
            else
            {
                this->distanceintegrate += this->d;
            }
        }
        else if(this->distanceintegrate < -100)
        {
            if(this->d < 0)
            {
                this->distanceintegrate += 0;
            }
            else
            {
                this->distanceintegrate += this->d;
            }
        }
        else {
            this->distanceintegrate += this->d;
        }
    }
    else this->distanceintegrate = 0;

    return this->distanceintegrate;
}

void traversalcontrol::setPidFactor(int chose, double p, double i, double d) {
    if (chose == 0) {
        this->pidfactor->pfactor = p;
        this->pidfactor->ifactor = i;
        this->pidfactor->dfactor = d;
    }
    if (chose == 1) {
        this->ppursuitfactor->Ld = p;
        this->ppursuitfactor->integratefactor = i;
    }
    if (chose == 2) {
        this->likepdfactor->kp = p;
        this->likepdfactor->ki = i;
        this->likepdfactor->kd = d;
    }
    if (chose == 3) {
        this->npdfactor->kp = p;
        this->npdfactor->ki = i;
        this->npdfactor->kd = d;
    }

}

double traversalcontrol::rbfpid()
{
	pidfactor->error = this->d;
	pidfactor->sumerror += this->d;
	//double fai = 0;
	//横线偏差相见做差分
	//fai = pidfactor->pfactor*pidfactor->error + pidfactor->ifactor*pidfactor->sumerror + pidfactor->dfactor*(pidfactor->error-pidfactor->lasterror);
	//航向角作为查分
	updatePID(0,pidfactor->error);

	double du=pidfactor->pfactor*xc[0]+pidfactor->dfactor*xc[1]+pidfactor->ifactor*xc[2];  
	u=u_1+du;  
	//u = pidfactor->pfactor*pidfactor->error + pidfactor->ifactor*pidfactor->sumerror + pidfactor->dfactor*(data::dnorth/180*M_PI);

	inx[0]=du;  
	inx[1]=pidfactor->error;  
	inx[2]=y_1;  

	u_1=u;  
	y_1=pidfactor->error;  

	for (int j=0;j<6;j++)  
	{  
		for(int i=0;i<3;i++)  
		{  
			ci_2[j][i]=ci_1[j][i];  
			ci_1[j][i]=ci[j][i];  
		}  

		bi_2[j]=bi_1[j];  
		bi_1[j]=bi[j];  

		w_2[j]=w_1[j];  
		w_1[j]=w[j];  
	}  

	xc[0]=pidfactor->error-pidfactor->lasterror;  
	xc[1]=pidfactor->error-2*pidfactor->lasterror+pidfactor->error_2;  
	xc[2]=pidfactor->error;
	pidfactor->error_2=pidfactor->lasterror;
	pidfactor->lasterror = pidfactor->error;

	kp_1=pidfactor->pfactor;  
	kd_1=pidfactor->dfactor;    
	ki_1=pidfactor->ifactor;   

	return u;
}

double traversalcontrol::caculateNorm2( double * tx,int count )
{
	double norm2=0;  
	for(int i=0;i<count;i++)  
	{  
		norm2+=pow(tx[i],2);  
	}  
	return norm2;  
}

double traversalcontrol::caculateHj( double* tx,double * tc,int count,double tbij )
{
	double norm2=0,hj=0;  
	for(int i=0;i<count;i++)  
	{  
		norm2+=pow((tx[i]-tc[i]),2);  
	}  
	//hj=exp(-norm2/(2*pow(tbj,2)));  
	return exp(-norm2/(2*pow(tbij,2)));  
}

double traversalcontrol::caculateYpY( double * tx,double* ty,int count )
{
	double re=0;  
	for (int i=0;i<count;i++)  
	{  
		re+=tx[i]*ty[i];  
	}  
	return re; 
}

void traversalcontrol::initPara()
{
	pidfactor->error=pidfactor->error_2=pidfactor->lasterror=y_1=0;  
	kp_1=0.2,ki_1=0.01,kd_1=1.2;  

	for (int i=0;i<6;i++)  
	{  
		bi[i]=10;  
		w[i]=0.1;
		h[i]=0;
		for (int j=0;j<3;j++)
		{
			ci[i][j]=0;
		}
	}
	for (int j=0;j<3;j++)
	{
		inx[j]=0;
		xc[j]=0;
	}

	memcpy(ci_1,ci,18*sizeof(double));  
	memcpy(ci_2,ci,18*sizeof(double));  
	memcpy(ci_3,ci,18*sizeof(double));  
	memcpy(w_1,w,6*sizeof(double));  
	memcpy(w_2,w,6*sizeof(double));  
	memcpy(w_3,w,6*sizeof(double));  
	memcpy(bi_1,bi,6*sizeof(double));  
	memcpy(bi_2,bi,6*sizeof(double));  
	memcpy(bi_3,bi,6*sizeof(double));  
	u_1=0;  
	y_1=0; 
	pidfactor->lasterror=0;
	pidfactor->error_2=0;
}

void traversalcontrol::updatePID( double yd,double ty )
{
	for(int j=0;j<6;j++)  
	{  
		h[j]=caculateHj(inx,ci[j],3,bi[j]);  
	}  
	double ym=caculateYpY(w,h,6);  

	double d_w[6]={0};  
	for (int j=0;j<6;j++)  
	{  
		d_w[j]=eta*(ty-ym)*h[j];  
	}  
	for (int i=0;i<6;i++)  
	{  
		w[i]=w_1[i]+d_w[i]+alfa*(w_1-w_2);  
	}  

	double d_bi[6]={0};  
	for (int j=0;j<6;j++)  
	{  
		double x_cij[3];  
		for(int i=0;i<3;i++)  
		{  
			x_cij[i]=inx[i]-ci[j][i];  
		}  
		d_bi[j]=eta*(ty-ym)*w[j]*h[j]*pow(bi[j],-3)*caculateNorm2(x_cij,3);  
	}  
	for (int i=0;i<6;i++)  
	{  
		bi[i]=bi_1[i]+d_bi[i]+alfa*(bi_1[i]-bi_2[i]);  
	}  

	double d_ci[6][3]={0};  
	for (int j=0;j<6;j++)  
	{  
		for (int i=0;i<3;i++)  
		{  
			d_ci[j][i]=eta*(pidfactor->error-ym)*w[j]*h[j]*(inx[i]-ci[j][i])*pow(bi[j],-2);  
		}  
	}  
	for (int i=0;i<3;i++)  
	{  
		for (int j=0;j<6;j++)  
		{  
			ci[j][i]=ci_1[j][i]+d_ci[j][i]+alfa*(ci_1[j][i]-ci_2[j][i]);  
		}  
	}  

	double yu=0;  
	for (int j=0;j<6;j++)  
	{  
		yu+=w[j]*h[j]*(ci[j][0]-inx[0])/pow(bi[j],2);  
	}  
	double dyu=yu;  
	//error=yd-ty;  
	pidfactor->pfactor=kp_1-etaP*pidfactor->error*dyu*xc[0];  
	pidfactor->dfactor=kd_1-etaD*pidfactor->error*dyu*xc[1];  
	pidfactor->ifactor=ki_1-etaI*pidfactor->error*dyu*xc[2];  
	if (pidfactor->pfactor<0)  
	{  
		pidfactor->pfactor=0;  
	}  
	if (pidfactor->ifactor<0)  
	{  
		pidfactor->ifactor=0;  
	}  
	if (pidfactor->dfactor<0)  
	{  
		pidfactor->dfactor=0;  
	}  
}

