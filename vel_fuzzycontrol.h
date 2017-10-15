#ifndef VEL_FUZZYCONTROL_H
#define VEL_FUZZYCONTROL_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

/* handy macros */
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define NUM_RULES	15

enum {
    /* the membership wedges of distance */
    NLd = 10,
    NSd = 11,
    Zd  = 12,
    PSd = 13,
    PLd = 14,

    /* the membership wedges of theta */
    NLTheta = 20,
    NSTheta = 21,
    ZTheta = 22,
    PSTheta = 23,
    PLTheta = 24
};

typedef struct {
    double value;	/* crisp value */
    int membership_func;    /* the specific function Small, Large ... */
}fuzzy_in;

class vel_fuzzycontrol
{
public:
    vel_fuzzycontrol();
    double fuzzyOutput(double d, double yaw);

    static double velocity[4];
    static int rule[5][5];

    //输入输出权重因子
    double wd;
    double wtheta;
    double wv;

private:
    double compute_aggregation(double memb_val_in1, double memb_val_in2);
    double compute_membership(fuzzy_in* in);
};

#endif // VEL_FUZZYCONTROL_H
