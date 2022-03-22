#ifndef __BSP_CAN
#define __BSP_CAN
#include "main.h"
#include "can.h"

/* �������ID ����*/
//��ʱ��6020����id3412
#define	CAN1_M3508_1 0x201	// ����1 id1
#define	CAN1_M3508_2 0x202	// ����2 id2

//��ʱ��3508����id1212

#define CAN2_M3508_3  0x201	// ����3   id1
#define CAN2_M3508_4  0x202	// ����4   id2
#define CAN2_GM6020_3  0x205	// ����3   id1
#define CAN2_GM6020_4  0x206	// ����4   id2
#define	CAN2_GM6020_1 0x207	// ����1     id3
#define	CAN2_GM6020_2 0x208	// ����2     id4


#define FWAngle2Angle(x) ((x)/8191.0*360)
#define Angle2FWAngle(x) ((x)/360.0*8191)

/* ����3508״̬�ṹ�壬�洢��ǰ3508�ķ���ֵ*/
typedef struct {
    int16_t  	set_current;	// д��ĵ���
    uint16_t 	angle;			// �Ƕ�
	  int16_t  	last_angle;
	  float 		total_angle;
    int16_t 	speed;			// �ٶ�
    int16_t		current;		// ����
    uint8_t 	temperature;	// �¶�
} CAN_M3508_TypeDef;


/* ����6020״̬�ṹ�壬�洢��ǰ6020�ķ���ֵ*/
typedef struct
{
  int16_t  set_voltage;	// д��ĵ�ѹ
  int16_t angle;			// �Ƕ�
	int16_t  	last_angle;
	float 		total_angle;
  int16_t  speed;         // �ٶ�
  int16_t  current;       // ����
  int8_t  temperature;   // �¶�
	float set_angle_speed;
} CAN_GM6020_TypeDef;

/* ����2006״̬�ṹ�壬�洢��ǰ6020�ķ���ֵ*/
typedef struct
{
    int16_t  set_current;	// д��ĵ���
    uint16_t angle;			// �Ƕ�
	  int16_t  	last_angle;
	  float 		total_angle;
    int16_t  speed;         // �ٶ�
    int16_t  current;       // ����
    uint8_t  temperature;   // �¶�
} CAN_M2006_TypeDef;

extern CAN_GM6020_TypeDef 	CAN_GM6020[4];	// ��̨���
//extern CAN_M2006_TypeDef 	CAN_M2006[2]	;	// ���̵��
extern CAN_M3508_TypeDef 	CAN_M3508[7];	// ǰ�ĸ��ǵ��̵�� ��������Ħ���ֵ�� ���һ�����̵��
extern float powerData[4];
extern int32_t percent;
void CAN_FilterInit(CAN_HandleTypeDef* hcan);
uint8_t CAN_SendMsg(CAN_HandleTypeDef* hcan,uint8_t ide,uint32_t id,uint8_t len,uint8_t *data);
void CAN_M3508_SetCurrent(int16_t i1,int16_t i2,int16_t i3,int16_t i4);

void get_total_angle(CAN_M3508_TypeDef *p);
void get_total_angle_2006(CAN_M2006_TypeDef *p);
void get_total_angle_6020(CAN_GM6020_TypeDef *p);

void CAN_Superpower(uint16_t temPower);
void CAN_Chassis_SendCurrent(void);
void CAN_Gimbal_SendVoltage(void);
void CAN_Shoot_SendCurrent(void);
	
void get_total_C_yaw(void);
void CAN_TA(void);
#endif


