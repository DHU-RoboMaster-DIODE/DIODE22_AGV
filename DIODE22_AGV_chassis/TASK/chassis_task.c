/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  */
#include "chassis_task.h"
#include "rc_control.h"
#include "arm_math.h"

uint16_t time_chassis;
float chassis_power_limit;
uint8_t flag_upup;

chassis_speed_t chassis_speed;
//�����˶�
uint16_t FollowInitAngle=5312; //4216
uint16_t classis_speed,spin_speed;   //���̻����ٶ�,С�����ٶȸ��ݲ���ϵͳ�ı�
uint8_t SpinStartFlag,IsSpinFlag;//С���ݱ�־
uint8_t FollowSwitchFlag,IsFollowFlag=1,LastIsFollowFlag=1;
/**
  * @brief          ���̿��ƺ���
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_rc_ctrl(void);
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void Chassis_Power_Limit(void);	
	
	
void chassis_task(void const *pvParameters)
{
    //����һ��ʱ��
    osDelay(CHASSIS_TASK_INIT_TIME);
		classis_speed=1000;//�ƶ��ٶ�
		spin_speed=1000;//С�����ٶ�
	  for(int i=0; i<4; i++)	// �ĸ����̵��
    {
        PID_Init(&PID_M3508[i],POSITION_PID,16000,16000,2,0,1.5);//5,0.01
			  PID_Init(&PID_GM6020[i],POSITION_PID,10000,10000,50,0.001,150);
////			  PID_Init(&PID_GM6020_speed[i],POSITION_PID,30000,30000,100,0,10);
    }
    static portTickType lastWakeTime;  
		lastWakeTime = xTaskGetTickCount(); 
    while (1)
    {
				time_chassis++;
				if(rc_flag)
				{
				
					chassis_rc_ctrl() ;
					Chassis_Power_Limit();
				}
				else    //��ң����ʧ�عر���յ���ֵ
				{
					  CAN_M3508[0].set_current=0;
            CAN_M3508[1].set_current=0;
            CAN_M3508[2].set_current=0;
            CAN_M3508[3].set_current=0;
					  CAN_GM6020[0].set_voltage=0;
					  CAN_GM6020[1].set_voltage=0;
					  CAN_GM6020[2].set_voltage=0;
					  CAN_GM6020[3].set_voltage=0;
				}

				//CANͨ��������ĸ�������͵���ֵ
				CAN_Chassis_SendCurrent();
        //ϵͳ������ʱ
        osDelayUntil(&lastWakeTime,CHASSIS_CONTROL_TIME_MS);
    }
}
	/*************************�����˶�*******************************/
float Vx,Vy,Vw;
float setAngle[4];//6020�趨�Ƕ�
double Radius=1.0;//Խ����ҡ�˿���Խǿ     
int flag_a[4]={1,1,1,1};
int drct[4]={1,1,1,1};//3508��ת����
double angle_temp[4];  
double CHASSIS_6020_Y_ANGLE[4]={1790,3737,1902,1617};//6020��ʼ�Ƕȣ���ʱ3508ȫ����ǰ��������ǰΪ��
double wheel_angle_last[4]={1790,3737,1902,1617};    //
int flag_drct=1;

void calc_3508_speed(void);//3508�ٶȽ���
void calc_6020_angle(void);//6020�ǶȽ���
float Find_min_Angle(float angle1,float angle2);
void AngleLoop_int(int16_t *a,int n);
void AngleLoop_f(double *a,int n);

void chassis_rc_ctrl(void)
{  

      Vx=-(double)chassis_speed.vy/10.0f;
			Vy=(double)chassis_speed.vx/10.0f;    
      Vw=(double)chassis_speed.wz;    
		
		calc_3508_speed();
		calc_6020_angle();
		
//		for(int i=0;i<4;i++){
//			if(fabs(Find_min_Angle(CAN_GM6020[i].angle,setAngle[i]) ) > 2048 ){
//				setAngle[i]+=4096;	 
//        if(flag_a[i])	{			
//				  drct[i] = -drct[i];		
//				  flag_a[i]=0;
//			  }	
//			}
//			else flag_a[i]=1;
//			
//	}
//	
//  for(int i=0;i<4;i++){//�ػ�,���ڱ仯��4096����������һ��
//			for( ; setAngle[i] > 8191; )setAngle[i] -= 8191;
//	    for( ; setAngle[i] < 0; )setAngle[i]+= 8191;
//		}  
//		
  static float setAngle_min[4];
  if(Vx == 0 && Vy == 0 && Vw == 0)//ҡ�˻���,����ԭ�Ƕ�
  {
    for(int i=0;i<4;i++)
     setAngle[i] = wheel_angle_last[i];
  }
//  else
//  {
    for(int i=0;i<4;i++)
		  wheel_angle_last[i] = CAN_GM6020[i].angle;
//  }   

		for(int i=0;i<4;i++){
	           setAngle_min[i]=Find_min_Angle(setAngle[i], CAN_GM6020[i].angle);
               if(-4096<=setAngle_min[i] && setAngle_min[i]<-2048) drct[i]=-1,setAngle_min[i]+=4096;
               else if(2048<setAngle_min[i] && setAngle_min[i]<=4096) drct[i]=-1,setAngle_min[i]-=4096;
               else drct[i]=1;	
			   CAN_GM6020[i].set_voltage =  PID_Calculate(&PID_GM6020[i],setAngle_min[i] ,0);	
			   if(setAngle_min[i]<100)CAN_M3508[i].set_current  =   PID_Calculate(&PID_M3508[i], (drct[i])*(setSpeed[i]), CAN_M3508[i].speed);
		}
		if(flag_drct){
			for(int i=0;i<4;i++)drct[i]=1;
			flag_drct=0;
		}
	}



void calc_3508_speed(){
	
	  double  k=1000;//
	  setSpeed[0] = sqrt(	pow(Vy - Vw * Radius * 0.707107f,2)
                       +	pow(Vx - Vw * Radius * 0.707107f,2)
                       ) * k;
    setSpeed[1] = sqrt(	pow(Vy - Vw * Radius * 0.707107f,2)
                       +	pow(Vx + Vw * Radius * 0.707107f,2)
                       ) * k;
    setSpeed[2] = sqrt(	pow(Vy + Vw * Radius * 0.707107f,2)
                       +	pow(Vx + Vw * Radius * 0.707107f,2)
                       ) * k;
    setSpeed[3] = sqrt(	pow(Vy + Vw * Radius * 0.707107f,2)
                       +	pow(Vx - Vw * Radius * 0.707107f,2) 
                       ) * k;
	  if(Vx == 0 && Vy == 0 && Vw == 0)//ҡ�˻���ʱ
  {
    for(int i=0;i<4;i++)
     setSpeed[i] = 0;
  }	
}


void calc_6020_angle(){

    double atan_angle[4]; 

	  //6020Ŀ��Ƕȼ���
	if((( Vy - Vw*Radius*0.707107f)==0)&&((Vy + Vw*Radius*0.707107f)==0)){
	  if((Vx - Vw*Radius*0.707107f)==0  && (Vx - Vw*Radius*0.707107f)==0){
	  atan_angle[0]=0;
	  atan_angle[1]=0;			
	  atan_angle[2]=0;
	  atan_angle[3]=0;		  
	  }
	  else{
	  atan_angle[0]=( Vx - Vw*Radius*0.707107f)>0?90:-90;
	  atan_angle[1]=( Vx + Vw*Radius*0.707107f)>0?90:-90;			
	  atan_angle[2]=( Vx + Vw*Radius*0.707107f)>0?90:-90;
	  atan_angle[3]=( Vx - Vw*Radius*0.707107f)>0?90:-90;		
	  }
	 }
	else if(( Vy - Vw*Radius*0.707107f)==0){
	  atan_angle[0]=( Vx - Vw*Radius*0.707107f)>0?90:-90;
	  atan_angle[1]=( Vx + Vw*Radius*0.707107f)>0?90:-90;	
      atan_angle[2]=atan2(( Vx + Vw*Radius*0.707107f),(Vy + Vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[3]=atan2(( Vx - Vw*Radius*0.707107f),(Vy + Vw*Radius*0.707107f))*180.0f/PI;		
	}
	else if(( Vy + Vw*Radius*0.707107f)==0){
      atan_angle[0]=atan2(( Vx - Vw*Radius*0.707107f),(Vy - Vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[1]=atan2(( Vx + Vw*Radius*0.707107f),(Vy - Vw*Radius*0.707107f))*180.0f/PI;	
	  atan_angle[2]=( Vx + Vw*Radius*0.707107f)>0?90:-90;
	  atan_angle[3]=( Vx - Vw*Radius*0.707107f)>0?90:-90;		
	}
    else if(!((( Vy - Vw*Radius*0.707107f)==0)&&((Vy + Vw*Radius*0.707107f)==0)))//��ֹ��Ϊ0
    {
      atan_angle[0]=atan2(( Vx - Vw*Radius*0.707107f),(Vy - Vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[1]=atan2(( Vx + Vw*Radius*0.707107f),(Vy - Vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[2]=atan2(( Vx + Vw*Radius*0.707107f),(Vy + Vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[3]=atan2(( Vx - Vw*Radius*0.707107f),(Vy + Vw*Radius*0.707107f))*180.0f/PI;	
    }  
		setAngle[0] = CHASSIS_6020_Y_ANGLE[0] + (atan_angle[0]*22.75);//22.75Ϊÿ�仯1�ȱ�����ֵ�ı仯
		setAngle[1] = CHASSIS_6020_Y_ANGLE[1] + (atan_angle[1]*22.75);//�� 8192/360.0
		setAngle[2] = CHASSIS_6020_Y_ANGLE[2] + (atan_angle[2]*22.75);
		setAngle[3] = CHASSIS_6020_Y_ANGLE[3] + (atan_angle[3]*22.75);

		for(int i=0;i<4;i++){//�ػ�
			for( ; setAngle[i] > 8191; )setAngle[i] -= 8191;
	    for( ; setAngle[i] < 0; )setAngle[i]+= 8191;
		}  

}
float Find_min_Angle(float angle1,float angle2)
{
	float temp;
    temp = angle1 - angle2;
    if(temp > 4096)
       temp-=8192;
	if(temp<-4096)
	   temp+=8192;	
    return temp;
}


void Chassis_Power_Limit(void)
{	
	float    kLimit = 1;//��������ϵ��
	float    chassis_totaloutput = 0;//ͳ�����������
  static float fTotalCurrentLimit;
	
	//ͳ�Ƶ��������
	chassis_totaloutput = abs(CAN_M3508[0].current) + abs(CAN_M3508[1].current)
							+ abs(CAN_M3508[2].current) + abs(CAN_M3508[3].current);
	
  if(powerData[1]>18) fTotalCurrentLimit=(float)(chassis_power_limit/powerData[1])*1000*kLimit;
  else fTotalCurrentLimit=chassis_power_limit/18.0f*1000*kLimit;
	if(powerData[1]<=16){
    fTotalCurrentLimit*=0.95f;
	}
	else if(flag_upup){
			fTotalCurrentLimit=65535;
	}
  else if(!flag_upup){
	}
	
	//���̸�����������·���
	if (chassis_totaloutput > fTotalCurrentLimit)
	{
		CAN_M3508[0].current = (int16_t)(CAN_M3508[0].current / chassis_totaloutput * fTotalCurrentLimit);
		CAN_M3508[1].current = (int16_t)(CAN_M3508[1].current / chassis_totaloutput * fTotalCurrentLimit);
		CAN_M3508[2].current = (int16_t)(CAN_M3508[2].current / chassis_totaloutput * fTotalCurrentLimit);
		CAN_M3508[3].current = (int16_t)(CAN_M3508[3].current / chassis_totaloutput * fTotalCurrentLimit);	
	}
}

