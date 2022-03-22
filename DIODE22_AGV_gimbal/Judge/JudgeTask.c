	/**
  ******************************************************************************
  * File Name          : JudgeTask.c
  * Description        : 裁判系统处理任务，得到裁判系统信息
  ******************************************************************************
  *
  * Copyright (c) 2019 Team 
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "JudgeTask.h"
#include "mycrc.h"
#include <stdlib.h>
#include <string.h>    
#include "usart.h"
#include "bsp_can.h"

ext_game_states_t game_states;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP;
ext_dart_states_t dart_states;
ext_ICRA_buff_debuff_zone_states_t ICRA_buff_debuff_zone_states;
ext_event_data_t event_data;
ext_supply_projectile_action_t supply_projectile_action;
ext_referee_warning_t referee_warning;
ext_dart_remaining_time_t dart_remaining_time;
ext_game_robot_states_t game_robot_states;
ext_power_heat_data_t power_heat_data;
ext_game_robot_pos_t game_robot_pos;
ext_buff_musk_t buff_musk;
aerial_robot_energy_t robot_energy;
ext_robot_hurt_t robot_hurt;
ext_shoot_data_t shoot_data;
ext_bullet_remaining_t bullet_remaining;
ext_rfid_states_t rfid_states;
ext_dart_client_cmd_t dart_client_cmd;
ext_student_interactive_header_data_t student_interactive_header_data;

float2uchar ph01;
ext_SendClientData_graphic_t  GraphicData;
ext_SendClientData_character_t CharacterData;
ext_client_state_t  client_state_now;
ext_client_state_t  client_state_past;


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_states, 0, sizeof(game_states));
    memset(&game_result, 0, sizeof(game_result));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP));
    
    memset(&dart_states, 0, sizeof(dart_states));
    memset(&ICRA_buff_debuff_zone_states, 0, sizeof(ICRA_buff_debuff_zone_states));
    memset(&event_data, 0, sizeof(event_data));
  

    memset(&supply_projectile_action, 0, sizeof(supply_projectile_action));
    memset(&referee_warning, 0, sizeof(referee_warning));
    memset(&dart_remaining_time, 0, sizeof(dart_remaining_time));
    memset(&game_robot_states, 0, sizeof(game_robot_states));
	  memset(&power_heat_data, 0, sizeof(power_heat_data));
    memset(&game_robot_pos, 0, sizeof(game_robot_pos));
    memset(&buff_musk, 0, sizeof(buff_musk));
    memset(&robot_energy, 0, sizeof(robot_energy));
    memset(&robot_hurt, 0, sizeof(robot_hurt));
    memset(&shoot_data, 0, sizeof(shoot_data));

    memset(&bullet_remaining, 0, sizeof(bullet_remaining));
    memset(&rfid_states, 0, sizeof(rfid_states));
    memset(&dart_client_cmd, 0, sizeof(dart_client_cmd));
		memset(&student_interactive_header_data,0,sizeof(student_interactive_header_data));

}
void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATES_CMD_ID:
        {
            memcpy(&game_states, frame + index, sizeof(game_states));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP));
        }
        break;
        case DART_STATES_CMD_ID:
		    {
            memcpy(&dart_states, frame + index, sizeof(dart_states));
        }
				break;
        case ICRA_BUFF_DEBUFF_ZONE_STATES_CMD_ID:
				{
            memcpy(&ICRA_buff_debuff_zone_states, frame + index, sizeof(ICRA_buff_debuff_zone_states));
        }	
				break;
        case EVENT_DATA_CMD_ID:
        {
            memcpy(&event_data, frame + index, sizeof(event_data));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action, frame + index, sizeof(supply_projectile_action));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning, frame + index, sizeof(referee_warning));
        }
        break;
        case DART_REMAINING_TIME_CMD_ID:
				{
            memcpy(&dart_remaining_time, frame + index, sizeof(dart_remaining_time));
        }
        break;	
        case  GAME_ROBOT_STATES_CMD_ID:
        {
            memcpy(&game_robot_states, frame + index, sizeof(game_robot_states));
        }
        break;
				case  POWER_HEAT_DATA_CMD_ID:
				{
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data));
        }
        break;
				case  GAME_ROBOT_POS_CMD_ID:
				{
            memcpy(&game_robot_pos, frame + index, sizeof(game_robot_pos));
        }
        break;
        case  BUFF_MUSK_CMD_ID:
				{
            memcpy(&buff_musk, frame + index, sizeof(buff_musk));
        }
        break;
        case ROBOT_ENERGY_CMD_ID :
        {
            memcpy(&robot_energy, frame + index, sizeof(robot_energy));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt, frame + index, sizeof(robot_hurt));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining, frame + index, sizeof(bullet_remaining));
        }
        break;
				case RFID_STATES_CMD_ID:
			  {
            memcpy(&rfid_states, frame + index, sizeof(rfid_states));
        }
        break;	
				case  DART_CLIENT_CMD_CMD_ID:
			  {
            memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd));
        }
        break;
        case STUDENT_INTERACTIVE_HEADER_DATA_CMD_ID:
        {
            memcpy(&student_interactive_header_data, frame + index, sizeof(student_interactive_header_data));
        }
        break;
        default:
        {
            break;
        }
    }
}





