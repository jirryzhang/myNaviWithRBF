#include "vel_fuzzycontrol.h"
#include "data.h"

double vel_fuzzycontrol::velocity[4] = {0, 1.0/2, 2.0/2};
/*the fuzzy rules
*Z is the 1
*S is the 2
*M is the 3
*L is the 4
*/
int vel_fuzzycontrol::rule[5][5] = {{1,1,1,2,2},{0,1,0,1,1},{0,0,2,0,0},{1,1,0,1,1},{2,2,1,1,1}};

vel_fuzzycontrol::vel_fuzzycontrol()
{
    wd = 5;
    wtheta = 1.0/(20/180*M_PI);
    wv = 1.0;

}

double vel_fuzzycontrol::fuzzyOutput(double d, double yaw) {
    //模糊速度控制
//    fuzzy_in *d_in = (fuzzy_in*)calloc(1, sizeof(fuzzy_in));
//    fuzzy_in *v_in = (fuzzy_in*)calloc(1, sizeof(fuzzy_in));
//    d_in->value = d*wd;
//    v_in->value = v*wv;

//    double w[NUM_RULES] = {0.0f};
//    double u[NUM_RULES] = {0.0f};

//    int k = 0;
//    double sum_w = 0.0f, sum_u = 0.0f;
//    for (int i=10; i<=14; i++) {
//        for (int j=20; j<=24; j++) {
//            d_in->membership_func = i;
//            v_in->membership_func = j;
//            w[k] = compute_aggregation(compute_membership(d_in), compute_membership(v_in));
//            u[k] = velocity[rule[i-10][j-20]];
//            sum_w += w[k];
//            sum_u += w[k]*u[k];
//            k++;
//        }
//    }
//    double out = sum_u/sum_w;

//    free(d_in);
//    free(v_in);
//    return out*wd;

    //阶梯速度控制
    if(fabs(d)>=0.2 || fabs(data::theta + data::initTheta) > 8) {
        return 0.5;
    } else if(fabs(d) >= 0.10 && fabs(data::theta + data::initTheta)<7)
    {
        if(fabs(yaw) <8 ) {
            return 0.7;
        }
        else
            return 0.6;
    } else if(fabs(d) >= 0.08 && fabs(data::theta + data::initTheta)<6)
    {
        if(fabs(yaw) < 8)
            return 0.8;
        else
            return 0.7;
    } else if(fabs(d) >= 0.05 && fabs(data::theta + data::initTheta)<5)
    {
        if(fabs(yaw) < 8)
            return 0.9;
        else
            return 0.8;
    } else {
        if(fabs(yaw)<5)
            return 1;
        else
            return 0.9;
    }
}

double vel_fuzzycontrol::compute_aggregation(double memb_val_in1, double memb_val_in2) {
    return min(memb_val_in1, memb_val_in2);
}

double vel_fuzzycontrol::compute_membership(fuzzy_in *in) {

    double a, b, c;
    double out = 0.0f;

    switch(in->membership_func){
    /* for distance error*/
    case NLd:
        a = -1.5, b = -1, c = -0.5;
        break;
    case NSd:
        a = -1, b = -0.5, c = 0;
        break;
    case Zd:
        a = -0.5, b = 0, c = 0.5;
        break;
    case PSd:
        a = 0, b = 0.5, c = 1;
        break;
    case PLd:
        a = 0.5, b = 1, c = 1.5;
        break;
    case NLTheta:
        a = -1.5, b = -1, c = -0.5;
        break;
    case NSTheta:
        a = -1, b = -0.5, c = 0;
        break;
    case ZTheta:
        a = -0.5, b = 0, c = 0.5;
        break;
    case PSTheta:
        a = 0, b = 0.5, c = 1;
        break;
    case PLTheta:
        b = 0.5, b = 1, c = 1.5;
        break;

    }

    /*s使用等腰三角形为隶属度函数*/
    out = max(min((in->value - a)/(b-a), (c - in->value)/(c - b)), 0);
    if (in->value >= 1 || in->value <= -1)
        out = 1;
    return out;

}
