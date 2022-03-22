/**
  ******************************************************************************
  * File Name          : JudgeTask.h
  * Description        : 裁判系统处理任务，得到裁判系统信息
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __JUDGETASK_H
#define __JUDGETASK_H

#include <stdint.h>
#include "main.h"
typedef __packed struct {
    uint16_t stageRemainTime;
    uint8_t gameProgress;
    uint8_t robotLevel;
    uint16_t remainHP;
    uint16_t maxHP;
} extGameRobotState_t;

typedef __packed struct {
    uint8_t armorType : 4;
    uint8_t hurtType : 4;
} extRobotHurt_t;

typedef __packed struct {
    uint8_t bulletType;
    uint8_t bulletFreq;
    float bulletSpeed;
} extShootData_t;

typedef __packed struct {
    float chassisVolt;
    float chassisCurrent;
    float chassisPower;
    float chassisPowerBuffer;
    uint16_t shooterHeat0;
    uint16_t shooterHeat1;
} extPowerHeatData_t;

typedef __packed struct {
    uint8_t cardType;
    uint8_t cardIdx;
} extRfidDetect_t;

typedef __packed struct {
    uint8_t winner;
} extGameResult_t;

typedef __packed struct {
    uint8_t buffType;
    uint8_t buffAddition;
} extGetBuff_t;

typedef __packed struct {
    float x;
    float y;
    float z;
    float yaw;
} extGameRobotPos_t;

typedef __packed struct {
    float data1;
    float data2;
    float data3;
    uint8_t mask;
} extShowData_t;

/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;

typedef enum {
    ONLINE = 0,
    OFFLINE = 1,
} JudgeState_e;

void judgeUartRxCpltCallback(void);
void InitJudgeUart(void);

extern void getJudgeState(void);
extern extGameRobotState_t RobotState;
extern extShootData_t ShootData0;
extern extShootData_t ShootData1;
extern JudgeState_e JUDGE_State;



#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)



typedef enum
{
    GAME_STATES_CMD_ID                         = 0x0001,
    GAME_RESULT_CMD_ID                         = 0x0002,
    GAME_ROBOT_HP_CMD_ID                       = 0x0003,
	  DART_STATES_CMD_ID                         = 0x0004,
	  ICRA_BUFF_DEBUFF_ZONE_STATES_CMD_ID        = 0X0005,
    EVENT_DATA_CMD_ID                          = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID            = 0x0102,
    REFEREE_WARNING_CMD_ID                     = 0x0104,
	  DART_REMAINING_TIME_CMD_ID                 = 0x0105,
	  GAME_ROBOT_STATES_CMD_ID                   = 0x0201,
    POWER_HEAT_DATA_CMD_ID                     = 0x0202,
    GAME_ROBOT_POS_CMD_ID                      = 0x0203,
    BUFF_MUSK_CMD_ID                           = 0X0204,
    ROBOT_ENERGY_CMD_ID                        = 0x0205,
    ROBOT_HURT_CMD_ID                          = 0x0206,
    SHOOT_DATA_CMD_ID                          = 0x0207,
    BULLET_REMAINING_CMD_ID                    = 0x0208,
	  RFID_STATES_CMD_ID                         = 0x0209,
	  DART_CLIENT_CMD_CMD_ID                     = 0x020A,
    STUDENT_INTERACTIVE_HEADER_DATA_CMD_ID     = 0x0301,
    IDCustomData,
}referee_cmd_id_t;
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)


//RM2019
//比赛状态数据：0x0001。发送频率：1Hz
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
uint64_t SyncTimeStamp;
} ext_game_states_t;
//比赛结果数据：0x0002。发送频率：比赛结束后发送
typedef __packed struct
{
 uint8_t winner;
} ext_game_result_t;
//机器人血量数据：0x0003。发送频率：1Hz
typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP;
 uint16_t red_3_robot_HP;
 uint16_t red_4_robot_HP;
 uint16_t red_5_robot_HP;
 uint16_t red_7_robot_HP;
uint16_t red_outpost_HP;
 uint16_t red_base_HP;
 uint16_t blue_1_robot_HP;
 uint16_t blue_2_robot_HP;
 uint16_t blue_3_robot_HP;
 uint16_t blue_4_robot_HP;
 uint16_t blue_5_robot_HP;
 uint16_t blue_7_robot_HP;
uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
} ext_game_robot_HP_t;
//飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人
typedef __packed struct
{
 uint8_t dart_belong;
 uint16_t stage_remaining_time;
} ext_dart_states_t;
//人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz 周期发送，发送范围：所有机器人
typedef __packed struct
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3;
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3;
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3;
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3;
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3;
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
uint16_t blue1_bullet_left;
uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_states_t;
//场地事件数据：0x0101。发送频率：事件改变后发送
typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;

//补给站动作标识：0x0102。发送频率：动作改变后发送, 发送范围：己方机器人
typedef __packed struct
{
 uint8_t supply_projectile_id;
 uint8_t supply_robot_id;
 uint8_t supply_projectile_step;
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送
typedef __packed struct
{
 uint8_t level;
 uint8_t foul_robot_id;
} ext_referee_warning_t;

//飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人
typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

//比赛机器人状态：0x0201。发送频率：10Hz
typedef __packed struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
	
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
	
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
	  uint16_t chassis_power_limit;
	  uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_states_t;

//实时功率热量数据：0x0202。发送频率：50Hz
typedef __packed struct {
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
	  uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;	
} ext_power_heat_data_t;

//机器人位置：0x0203。发送频率：10Hz
typedef __packed struct {
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

//机器人增益：0x0204。发送频率：状态改变后发送
typedef __packed struct {
    uint8_t power_rune_buff;
} ext_buff_musk_t;

//空中机器人能量状态：0x0205。发送频率：10Hz
typedef __packed struct {
    uint8_t energy_point;
    uint8_t attack_time;
} aerial_robot_energy_t;

//伤害状态：0x0206。发送频率：伤害发生后发送
typedef __packed struct {
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

//实时射击信息：0x0207。发送频率：射击后发送
typedef __packed struct {
    uint8_t bullet_type;
	  uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

//子弹剩余发射数：0x0208。发送频率：10Hz 周期发送，所有机器人发送
typedef __packed struct
{
 uint16_t bullet_remaining_num_17mm;
uint16_t bullet_remaining_num_42mm;
uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

//机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人
typedef __packed struct
{
 uint32_t rfid_status;
} ext_rfid_states_t;

//飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t dart_attack_target;
 uint16_t target_change_time;
 uint8_t first_dart_speed;
 uint8_t second_dart_speed;
 uint8_t third_dart_speed;
 uint8_t fourth_dart_speed;
 uint16_t last_dart_launch_time;
 uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

//交互数据接收信息：0x0301。发送频率：上限 10Hz
typedef __packed struct {
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

//客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz
typedef __packed struct {
    float data1;
    float data2;
    float data3;
    uint8_t masks;
} client_custom_data_t;

//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端

typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	client_custom_data_t  					clientData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_SendClientData_t;

/* 
	客户端删除图形 cmd_id 0x0301，内容 ID:0x0100
	发送频率：上限 10Hz  
*/

typedef __packed struct {
	uint8_t operate_type;
	uint8_t layer;
}ext_client_graphic_delete_t;


/* 
	图形数据 
*/

typedef __packed struct {
	uint8_t graphic_name[3];
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;	
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
}graphic_data_struct_t;

typedef __packed struct {
	uint8_t graphic_name[3];
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;	
	uint32_t start_y:11;
	int32_t  number;
}graphic_data_struct_int_t;

/* 
	客户端绘制一个图形 cmd_id 0x0301，内容 ID:0x0100
	发送频率：上限 10Hz  
*/
typedef __packed struct {
	graphic_data_struct_t graphic_data_struct;
}ext_client_custom_graphic_single_t;

typedef __packed struct {
	graphic_data_struct_t graphic_data_struct[7];
}ext_client_custom_graphic_number_t;

typedef __packed struct {
	graphic_data_struct_t graphic_data_struct;
	char data[30];
}ext_client_custom_character_t;

typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_graphic_number_t  		graphicData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_SendClientData_graphic_t;

typedef __packed struct
{
	xFrameHeader   					txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_character_t  		graphicData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_SendClientData_character_t;

typedef __packed struct {
  uint32_t aim:3;
	uint32_t speed_dial_shoot:3;      //档位
	uint32_t motion:3;                 //1 Move 2 Rotate 3 Follow        
	uint32_t wheel_of:3;               //0：off  1：on  2:error
	uint32_t capacity_percent:9;       // 0-100
	uint32_t bin_cover_of:3;           //0：off  1：on
	uint32_t IsSpeedUpFlag:8; 
	float InputPower;
}ext_client_state_t;

extern ext_power_heat_data_t power_heat_data;
extern ext_game_robot_states_t game_robot_states;
extern  ext_client_state_t  client_state_now;
extern  ext_client_state_t  client_state_past;
extern ext_SendClientData_character_t CharacterData;
extern ext_SendClientData_graphic_t GraphicData;
extern ext_power_heat_data_t PowerHeat;
extern ext_buff_musk_t BuffMask;
extern ext_shoot_data_t ShootData;
extern client_custom_data_t custom_data;

void init_referee_struct_data(void);
void referee_data_solve(uint8_t *frame);



#endif /*__ JUDGETASK_H */
