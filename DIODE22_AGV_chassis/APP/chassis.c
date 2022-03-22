/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       chassis.c
 * @brief     
 * @note        
 * @Version    V1.0.0
 * @Date       2021.5     
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
 
#include "math.h"
#include "chassis.h"
#include "stm32f4xx_hal.h"
//#include "bsp_imu.h"

struct {
    float PI;
    float POW2;
    float angle1,angle2,angle3;//��ʼ�Ƕȣ�Ԥ����Ƕȣ���ʱ�Ƕȱ���
    float angleM;//����������ͷ���������ļн�

    float angN;//Ŀǰʮ�ᴫ�������յ���ƫ����
    float lastx,lasty,x,y;//�����λ�Ʒ�������
    float x1,y1;//ʵ��ķ�������
    float velV,palstance,velA;//������̵�λ���ٶȣ��������ת�ٶȣ�����������ֽ��ٶ�
    float K;//������ת�����ֽ��ٶȵı���ϵ����K = POW2*R/r(RΪ�Խ��������־���Ķ���֮һ(33cm)��rΪ���ְ뾶7cm)
} BN;

float map(float i,float imin,float imax,float rmin,float rmax) {
    i = limit(i,imin,imax);
    return ((i - imin) * (rmax - rmin)/(imax - imin) + rmin);
}

float limit(float i,float min,float max) {
    if(i<min)i=min;
    if(i>max)i=max;
    return i;
}

void BNumberInit() {

    BN.PI = acos(-1.0);
    BN.POW2 = pow(2,0.5);
    BN.K = BN.POW2*33/7;

    BN.angle1 = 0;
    BN.angle2 = 0;
    BN.angle3 = 0;

    BN.angleM = 0;

    BN.angN = 0;

    BN.lastx = 0;
    BN.lasty = 0;

    BN.x = 0;
    BN.y = 0;

    BN.x1 = 0;
    BN.y1 = 0;

    BN.velV = 0;
    BN.palstance = 0;
    BN.velA = 0;
}

int setSpeed[4];	// �ĸ�������ٶ�

//��Ƭ��ÿ100�������/ˢ��һ�����ݣ��ڴ��ڼ��Լ��ֵԤ����㲹����ֵ10�Σ�������������100��

/*
���̵������
	ǰ
	1 2
	3 4
	��
*/
void velocity(float x,float y) { //velocity(λ��x��λ��y����ת���ٶ�)

    BNumberInit();

//	BN.angN = imu.yaw;//ֱ�Ӷ�ȡ�Ƕȼƣ�360�㣩
    BN.angle1 = atan(y/x) + BN.angleM;//����ȫ��Ĭ�ϻ�����
//    BN.angle2 = BN.angle1 + imu.yaw*BN.PI/180;
    BN.velV = pow(x*x+y*y,0.5);
    BN.velA = 0 * BN.K;//������Ҫ�Ƕ�PID�ջ�
    BN.x = BN.velV * cos(BN.angle2);
    BN.y = BN.velV * sin(BN.angle2);

    BN.lastx = BN.x;
    BN.lasty = BN.y;

//	x = 2*x-imu.mx;//λ����������Ҫ�ȼ��ٶȼƵ���
//	y = 2*y-imu.mx;

    setSpeed[0] = x+y+BN.velA;
    setSpeed[1] = x-y+BN.velA;
    setSpeed[2] = -x+y+BN.velA;
    setSpeed[3] = -x-y+BN.velA;
}

