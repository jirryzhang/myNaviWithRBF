/*
 * data.h
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#ifndef DATA_H_
#define DATA_H_

class data {
public:
	data();
	virtual ~data();
    static double GPStime;
	static double lat; //转换之后的纬度坐标
    static double lng;//转换之后的经度坐标
	static char satelite_num[16];//卫星数目
    static int num;     //差分状态信息，RTK固定为4
	static double altitude;  //海拔高度
    static double north;     //与真北方向夹角
	static double velocity;  //车的速度
	static char stop[16];    //停止指令
	static double theta;     //车轮转角信息
	static double car_length; //车身长度信息
    static int flagTemp;

    static int status;
    static double distance;
    static double distance_mid;
    static double calculFai;

    static double AX_coor;
    static double AX_coor_mid;
    static double AY_coor;
    static double AY_coor_mid;
    static double BX_coor;
    static double BX_coor_mid;
    static double BY_coor;
    static double BY_coor_mid;
    static int motor_vel;
    static double rolldegree;
    static double dnorth;

    static int turnFlag;
    static double velValue;      // 模糊控制速度的值
    static double inValue;
    static double x_mid;
    static double y_mid;
    static double xInit;
    static double yInit;

    static double Kp;
    static double Ki;
    static double Kd;
    static int algorithmchose;

    /* 地头直线点 */
    static double p1x;
    static double p1y;
    static double p2x;
    static double p2y;

    static double northvelocity;
    static int ABturnFlag;
    static double expectedCircle;
    static double diffTheta;
    static double initTheta;
    static double slopeAB;
    static double xaMid_coor;
    static double yaMid_coor;
    static double udistanceMid;
    static double udiffTheta;
    static double rollAngle;
    static double pitchAngle;
};

#endif /* DATA_H_ */
