/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             底盘控制任务
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
//底盘运动
uint16_t FollowInitAngle=5312; //4216
uint16_t classis_speed,spin_speed;   //底盘基本速度,小陀螺速度根据裁判系统改变
uint8_t SpinStartFlag,IsSpinFlag;//小陀螺标志
uint8_t FollowSwitchFlag,IsFollowFlag=1,LastIsFollowFlag=1;
/**
  * @brief          底盘控制函数
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_rc_ctrl(void);
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void Chassis_Power_Limit(void);	
	
	
void chassis_task(void const *pvParameters)
{
    //空闲一段时间
    osDelay(CHASSIS_TASK_INIT_TIME);
		classis_speed=1000;//移动速度
		spin_speed=1000;//小陀螺速度
	  for(int i=0; i<4; i++)	// 四个底盘电机
    {
        PID_Init(&PID_M3508[i],POSITION_PID,16000,16000,4,0.02,5,300,0);//5,0.01
			  PID_Init(&PID_GM6020[i],POSITION_PID,10000,10000,20,0,50,0,0.1);
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
				else    //若遥控器失控关闭清空电流值
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

				//CAN通信向底盘四个电机发送电流值
				CAN_Chassis_SendCurrent();
        //系统绝对延时
        osDelayUntil(&lastWakeTime,CHASSIS_CONTROL_TIME_MS);
    }
}
	/*************************底盘运动*******************************/
float Vx,Vy,Vw;
float setAngle[4];//6020设定角度
double Radius=1.0;//越大右摇杆控制越强     
int flag_a[4]={1,1,1,1};
int drct[4]={1,1,1,1};//3508反转因子
double angle_temp[4];  
double CHASSIS_6020_Y_ANGLE[4]={1797,3738,1780,1645};//6020初始角度，此时3508全朝向前方，且向前为正 1797, 3738, 5876, 1645
double wheel_angle_last[4]={1790,3737,1902,1617};    //
int flag_drct=1;

void calc_3508_speed(void);//3508速度解算
void calc_6020_angle(void);//6020角度解算
float Find_min_Angle(float angle1,float angle2);
void AngleLoop_int(int16_t *a,int n);
void AngleLoop_f(double *a,int n);
float setAngle_min[4];
void chassis_rc_ctrl(void)
{  

      Vx=-(double)chassis_speed.vy/12.0f;
			Vy=(double)chassis_speed.vx/12.0f;    
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
//  for(int i=0;i<4;i++){//回环,由于变化了4096，所以再来一遍
//			for( ; setAngle[i] > 8191; )setAngle[i] -= 8191;
//	    for( ; setAngle[i] < 0; )setAngle[i]+= 8191;
//		}  


  if(Vx == 0 && Vy == 0 && Vw == 0)//摇杆回中,保持原角度
  {
    for(int i=0;i<4;i++)
     setAngle[i] = wheel_angle_last[i];
;
  }
	  for(int i=0;i<4;i++)
		  wheel_angle_last[i] = CAN_GM6020[i].angle;
		for(int i=0;i<4;i++){
	           setAngle_min[i]=Find_min_Angle(setAngle[i], CAN_GM6020[i].angle);
               if(-4096<=setAngle_min[i] && setAngle_min[i]<-2048) drct[i]=-1,setAngle_min[i]+=4096;
               else if(2048<setAngle_min[i] && setAngle_min[i]<=4096) drct[i]=-1,setAngle_min[i]-=4096;
               else drct[i]=1;	
			   CAN_GM6020[i].set_voltage =  PID_Calculate(&PID_GM6020[i],setAngle_min[i] ,0);	
			   if(setAngle_min[i]<200)CAN_M3508[i].set_current  =   PID_Calculate(&PID_M3508[i], (drct[i])*(setSpeed[i]), CAN_M3508[i].speed);
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
	  if(Vx == 0 && Vy == 0 && Vw == 0)//摇杆回中时
  {
    for(int i=0;i<4;i++)
     setSpeed[i] = 0;
  }	
}


void calc_6020_angle(){

    double atan_angle[4]; 

	  //6020目标角度计算
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
    else if(!((( Vy - Vw*Radius*0.707107f)==0)&&((Vy + Vw*Radius*0.707107f)==0)))//防止都为0
    {
      atan_angle[0]=atan2(( Vx - Vw*Radius*0.707107f),(Vy - Vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[1]=atan2(( Vx + Vw*Radius*0.707107f),(Vy - Vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[2]=atan2(( Vx + Vw*Radius*0.707107f),(Vy + Vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[3]=atan2(( Vx - Vw*Radius*0.707107f),(Vy + Vw*Radius*0.707107f))*180.0f/PI;	
    }  
		setAngle[0] = CHASSIS_6020_Y_ANGLE[0] + (atan_angle[0]*22.75);//22.75为每变化1度编码器值的变化
		setAngle[1] = CHASSIS_6020_Y_ANGLE[1] + (atan_angle[1]*22.75);//即 8192/360.0
		setAngle[2] = CHASSIS_6020_Y_ANGLE[2] + (atan_angle[2]*22.75);
		setAngle[3] = CHASSIS_6020_Y_ANGLE[3] + (atan_angle[3]*22.75);

		for(int i=0;i<4;i++){//回环
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

  float    fTotalCurrentLimit;//计算的得到带的限制总输出电流
	float    chassis_3508_totaloutput = 0;//统计总输出电流
	float    chassis_6020_totaloutput = 0;
 void Chassis_Power_Limit(void)
{	
	float    kLimit = 0.9;//功率限制系数
	//统计底盘总输出
	chassis_3508_totaloutput= abs(CAN_M3508[0].set_current) + abs(CAN_M3508[1].set_current)
							+ abs(CAN_M3508[2].set_current) + abs(CAN_M3508[3].set_current);
	//计算限制电流
	chassis_6020_totaloutput=abs(CAN_GM6020[0].set_voltage) + abs(CAN_GM6020[1].set_voltage)
							+ abs(CAN_GM6020[2].set_voltage) + abs(CAN_GM6020[3].set_voltage);
	fTotalCurrentLimit=(float)(chassis_power_limit-chassis_6020_totaloutput/1500)/24.0f*3000;
  if(fTotalCurrentLimit<0) fTotalCurrentLimit=0;
	//放电策略，调整功率限制函数
	
	if(powerData[1]<=16){
    kLimit=0.25f*(powerData[1]-14.0f)+0.5f;
	}
	else if(flag_upup==1){
			kLimit=5;  //不限制电流
	}
  else{
		if(powerData[1]>20)kLimit=1.5;
		else if(powerData[1]<=20)kLimit=0.25f*(powerData[1]-18.0f)+0.5f;
	}
	fTotalCurrentLimit*=kLimit;
  if(fTotalCurrentLimit<0) fTotalCurrentLimit=0;
	//底盘各电机电流重新分配
	if (chassis_3508_totaloutput > fTotalCurrentLimit)
	{
		CAN_M3508[0].set_current = (int16_t)(CAN_M3508[0].set_current / chassis_3508_totaloutput * fTotalCurrentLimit);
		CAN_M3508[1].set_current = (int16_t)(CAN_M3508[1].set_current / chassis_3508_totaloutput * fTotalCurrentLimit);
		CAN_M3508[2].set_current = (int16_t)(CAN_M3508[2].set_current / chassis_3508_totaloutput * fTotalCurrentLimit);
		CAN_M3508[3].set_current = (int16_t)(CAN_M3508[3].set_current / chassis_3508_totaloutput * fTotalCurrentLimit);	
	}
}

