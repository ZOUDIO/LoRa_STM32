/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: contains all hardware driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /******************************************************************************
  * @file    bsp.h
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   contains all hardware driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H__
#define __BSP_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "hw.h"
#include "timeServer.h"

/* Includes ------------------------------------------------------------------*/
/* Defines ------------------------------------------------------------------*/
#define USE_5V_OUTPUT               0 //0 - Overwrite PB5 for other usage, 1 - Control 5V of LSN50

#define DS3231_ALWAYS_POWERED	      1
#define DS3231_USED_INTERRUPT 	    0
#define DS3231_SYNC_INTERNAL_RTC    0

#if DS3231_ALWAYS_POWERED
#define DS3231_PWR_PIN_ON()       (void)0
#define DS3231_PWR_PIN_OFF()      (void)0
#else
#define DS3231_PWR_PIN_ON()       HAL_GPIO_WritePin(DS3231_PWR_PORT, DS3231_PWR_PIN, GPIO_PIN_SET); \
                                  HAL_Delay(300);

#define DS3231_PWR_PIN_OFF()      HAL_GPIO_WritePin(DS3231_PWR_PORT, DS3231_PWR_PIN, GPIO_PIN_RESET)
#endif /* End of DS3231_ALWAYS_POWERED */

/* Exported types ------------------------------------------------------------*/

typedef struct{
	
  uint8_t   in1;/*GPIO Digital Input 0 or 1*/
	
	float temp1;//DS18B20-1

	float temp2;//DS18B20-2

	float temp3;//DS18B20-3
	
	float oil;  //oil float

	float ADC_1; //ADC1
	
	float ADC_2;  //ADC2	

	float temp_sht;
	
	float hum_sht;
	
	uint16_t illuminance;	
	
  uint16_t distance_mm;
	
	uint16_t distance_signal_strengh;
	
	int32_t Weight;

  /**more may be added*/
} sensor_t;

typedef struct{
	
	uint16_t firm_ver;
	
	uint8_t freq_band;
	
	uint8_t sub_band;
	
} device_t;

typedef struct{
	uint8_t set_hour;
	uint8_t set_minute;
}time_boundaries_t;

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern TimerEvent_t OffPumpTimer;
extern bool is_timelimit_active;
extern volatile uint32_t COUNT3; // Presense count
extern uint32_t pump_off_ms;  
extern time_boundaries_t time_low_limit;
extern time_boundaries_t time_high_limit;
extern bool is_timelimit_active;
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief  initialises the sensor
 *
 * @note
 * @retval None
 */
void  BSP_sensor_Init( void  );

/**
 * @brief  sensor  read. 
 *
 * @note none
 * @retval sensor_data
 */
void BSP_sensor_Read( sensor_t *sensor_data, uint8_t message);

void Device_status( device_t *device_data);
void Pump_ON(void);
void Pump_OFF(void);

bool BSP_RTC_SetTime(uint8_t hour, uint8_t min, uint8_t sec);
void BSP_RTC_GetTime(uint8_t* p_hour, uint8_t* p_min, uint8_t* p_sec);
bool BSP_RTC_SetDate(uint8_t dayofweek, uint8_t date, uint8_t month, uint16_t year);
void BSP_RTC_GetDate(uint8_t* p_dayofweek, uint8_t* p_date, uint8_t* p_month, uint16_t* p_year);
void BSP_RTC_SyncTime(void);
bool Is_Time_In_Boundaries(void);
bool Set_Time_Low_Limit(uint8_t hour, uint8_t min);
bool Set_Time_High_Limit(uint8_t hour, uint8_t min);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
