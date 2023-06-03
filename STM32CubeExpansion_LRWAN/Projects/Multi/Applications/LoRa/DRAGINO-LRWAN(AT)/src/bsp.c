 /******************************************************************************
  * @file    bsp.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   manages the sensors on the application
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
  
  /* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "hw.h"
#include "timeServer.h"
#include "bsp.h"
#include "delay.h"
#include "vcom.h"
#include "lora.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if defined(LoRa_Sensor_Node)
#include "ds18b20.h"
#include "oil_float.h"
#include "gpio_exti.h"
#include "sht20.h"
#include "sht31.h"
#include "pwr_out.h"
#include "ult.h"
#include "lidar_lite_v3hp.h"
#include "ds3231_for_stm32_hal.h"
#include "weight.h"
#include "iwdg.h"
#include "bh1750.h"
#include "tfsensor.h"
#endif
/* Private macro -------------------------------------------------------------*/
#define I2C_TIMING    0x10A13E56 /* 100 kHz with analog Filter ON, Rise Time 400ns, Fall Time 100ns */ 

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
bool bh1750flags=0;
uint8_t mode2_flag=0;
static uint16_t AD_code1=0;
static uint16_t AD_code2=0;
static uint16_t AD_code3=0;

#ifdef USE_SHT
uint8_t flags=0;
#endif

//static GPIO_InitTypeDef  GPIO_InitStruct;
extern uint16_t batteryLevel_mV;
extern uint16_t fire_version;
TimerEvent_t OffPumpTimer;

#ifdef USE_SHT
extern float sht31_tem,sht31_hum;
extern I2C_HandleTypeDef I2cHandle1;
extern I2C_HandleTypeDef I2cHandle2;
extern I2C_HandleTypeDef I2cHandle3;
tfsensor_reading_t reading_t;
#endif

extern uint8_t mode;
extern uint8_t inmode,inmode2,inmode3;
extern uint16_t power_time;
extern uint32_t COUNT,COUNT2;

time_boundaries_t time_low_limit = {0, 0};
time_boundaries_t time_high_limit = {0, 0};
bool is_timelimit_active = false;


/**
 * @brief  Init the DS3231 with VCC pin <-> PB14
 */
void BSP_RTC_Init(void)
{
	// Init I2C for DS3231
	HAL_I2C_MspInit(&I2cHandle3);
	I2cHandle3.Instance              = I2Cx;
	I2cHandle3.Init.Timing           = I2C_TIMING;
	I2cHandle3.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
	I2cHandle3.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
	I2cHandle3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	I2cHandle3.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
	I2cHandle3.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  
	I2cHandle3.Init.OwnAddress1      = 0;
	
	if(HAL_I2C_Init(&I2cHandle3) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	/* Enable the Analog I2C Filter */
	HAL_I2CEx_ConfigAnalogFilter(&I2cHandle3,I2C_ANALOGFILTER_ENABLE);

	#if !DS3231_ALWAYS_POWERED
	// Init PWR pin for DS3231
	GPIO_InitTypeDef GPIO_InitStruct={0};
	DS3231_PWR_CLK_ENABLE();
	GPIO_InitStruct.Pin = DS3231_PWR_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(DS3231_PWR_PORT, &GPIO_InitStruct );
	DS3231_PWR_PIN_ON();			// Power on RTC
	#endif /* End of !DS3231_ALWAY_POWER */

	DS3231_Init(&I2cHandle3);
	PPRINTF("\r\n Date: %04d-%02d-%02d \n Time: %02d:%02d:%02d %s %d.%02d\n", 
	DS3231_GetYear(), DS3231_GetMonth(), DS3231_GetDate(), 
	DS3231_GetHour(), DS3231_GetMinute(), DS3231_GetSecond(), ds3231_dayofweek[DS3231_GetDayOfWeek()-1],
	DS3231_GetTemperatureInteger(), DS3231_GetTemperatureFraction());

	#if DS3231_USE_INTERRUPT
	// Alarm pin
	DS3231_INT_CLK_ENABLE();
	GPIO_InitStruct.Pin = DS3231_INT_PIN;	
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DS3231_INT_PORT, &GPIO_InitStruct);
	
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	// Set alarm 1 to trigger every week at 00:01:00 on Monday
	DS3231_EnableAlarm1(DS3231_ENABLED);
	DS3231_SetAlarm1Mode(DS3231_A1_MATCH_S_M_H_DAY);
	DS3231_SetAlarm1Second(0);
	DS3231_SetAlarm1Minute(0);
	DS3231_SetAlarm1Hour(1);
	DS3231_SetAlarm1Date(DS3231_MONDAY);
	#endif /* End of DS3231_USE_INTERRUPT */

	BSP_RTC_SyncTime();
	DS3231_PWR_PIN_OFF();			// Put DS3231 into low power mod

}

void BSP_RTC_SetTime(uint8_t hour, uint8_t min, uint8_t sec)
{
	if(hour > 23 || min > 59 || sec > 59)
	{
		PPRINTF("Invalid time input!\r\n");
		return;
	}
	DS3231_PWR_PIN_ON();			// Power on RTC

	DS3231_SetFullTime(hour, min, sec);
	BSP_RTC_SyncTime();
	DS3231_PWR_PIN_OFF();			// Put DS3231 into low power mode
}

void BSP_RTC_GetTime(uint8_t* p_hour, uint8_t* p_min, uint8_t* p_sec)
{
	DS3231_PWR_PIN_ON();			// Power on RTC

	*p_hour = DS3231_GetHour();
	*p_min = DS3231_GetMinute();
	*p_sec = DS3231_GetSecond();

	DS3231_PWR_PIN_OFF();			// Put DS3231 into low power mode
	
}

void BSP_RTC_SetDate(uint8_t dayofweek, uint8_t date, uint8_t month, uint16_t year)
{
	if(dayofweek > 7 || date > 31 || month > 12 || year > 2099)
	{
		PRINTF("Invalid date/time input \r\n");
		return;
	}
	DS3231_PWR_PIN_ON();			// Power on RTC

	DS3231_SetFullDate(date, month, dayofweek, year);
	DS3231_PWR_PIN_OFF();			// Put DS3231 into low power mode
}

void BSP_RTC_GetDate(uint8_t* p_dayofweek, uint8_t* p_date, uint8_t* p_month, uint16_t* p_year)
{
	DS3231_PWR_PIN_ON();			// Power on RTC

	*p_dayofweek = DS3231_GetDayOfWeek();
	*p_date = DS3231_GetDate();
	*p_month = DS3231_GetMonth();
	*p_year = DS3231_GetYear();

	DS3231_PWR_PIN_OFF();			// Put DS3231 into low power mode
}

//Sync time with the internal RTC of the STM32
void BSP_RTC_SyncTime(void)
{
	// Set the RTC current date/time
	RTC_HandleTypeDef* internal_rtc_handle = GetRTCHandle();
	RTC_TimeTypeDef internal_rtc_time = {
		.Hours = DS3231_GetHour(),
		.Minutes = DS3231_GetMinute(),
		.Seconds = DS3231_GetSecond(),
	};

	HAL_RTC_SetTime(internal_rtc_handle, &internal_rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(internal_rtc_handle, &internal_rtc_time, RTC_FORMAT_BIN);
	PPRINTF("\r\n Current internal time: %02d:%02d:%02d\n", internal_rtc_time.Hours,
															internal_rtc_time.Minutes,
															internal_rtc_time.Seconds);
	RTC_DateTypeDef dummy_date = {0};
	HAL_RTC_GetDate(internal_rtc_handle, &dummy_date, RTC_FORMAT_BIN);
}

// Get time from the internal RTC of the STM32
void BSP_RTC_GetInternalTime(uint8_t* p_hour, uint8_t* p_min, uint8_t* p_sec)
{
	RTC_HandleTypeDef* internal_rtc_handle = GetRTCHandle();
	RTC_TimeTypeDef internal_rtc_time = {0};
	HAL_RTC_GetTime(internal_rtc_handle, &internal_rtc_time, RTC_FORMAT_BIN);
	*p_hour = internal_rtc_time.Hours;
	*p_min = internal_rtc_time.Minutes;
	*p_sec = internal_rtc_time.Seconds;
}

bool Is_Time_In_Boundaries(void)
{
	uint8_t cur_hour, cur_min, cur_sec;
	BSP_RTC_GetInternalTime(&cur_hour, &cur_min, &cur_sec);
	if(cur_hour >= time_low_limit.set_hour && cur_hour <= time_high_limit.set_hour)
	{
		if(cur_min >= time_low_limit.set_minute && cur_min <= time_high_limit.set_minute)
		return true;
	}
	return false;
}

bool Set_Time_Low_Limit(uint8_t hour, uint8_t min)
{
	if(hour > 23 || min > 59)
		return false;
	time_low_limit.set_hour = hour;
	time_low_limit.set_minute = min;
	return true;
}

bool Set_Time_High_Limit(uint8_t hour, uint8_t min)
{
	if(hour > 23 || min > 59)
		return false;
	time_high_limit.set_hour = hour;
	time_high_limit.set_minute = min;
	return true;
}


void Pump_ON(void)
{
	HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_SET);
}

void Pump_OFF(void)
{
	HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);
}

void BSP_Sensor_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();


	/* PROX_EN (PA10) pin as input */
	GPIO_InitStruct.Pin = GPIO_PIN_10;	
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* PROX_SIGNAL_3V3 PB14 as interrupt input */
	GPIO_InitStruct.Pin = GPIO_PIN_14;	
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void BSP_Button_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_5;	
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void BSP_Pump_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	PUMP_CLK_ENABLE();
	GPIO_InitStruct.Pin = PUMP_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(PUMP_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);

	// Timer
	TimerInit(&OffPumpTimer, Pump_OFF);
}


void BSP_sensor_Read( sensor_t *sensor_data, uint8_t message)
{	
 	#if defined(LoRa_Sensor_Node)

	HW_GetBatteryLevel( );	
	if(message==1)
	{
		PPRINTF("\r\n");
		PPRINTF("Bat:%.3f V\r\n",(batteryLevel_mV/1000.0));
		if(mode==6)
		{
			PPRINTF("PB14_count1:%u\r\n",COUNT);
		}
		else if(mode==7)
		{
			PPRINTF("PB14_status:%d\r\n",HAL_GPIO_ReadPin(GPIO_EXTI14_PORT,GPIO_EXTI14_PIN));
			PPRINTF("PB15_status:%d\r\n",HAL_GPIO_ReadPin(GPIO_EXTI15_PORT,GPIO_EXTI15_PIN));
			PPRINTF("PA4_status:%d\r\n",HAL_GPIO_ReadPin(GPIO_EXTI4_PORT,GPIO_EXTI4_PIN));
		}
		else if(mode==9)
		{
			PPRINTF("PB14_count1:%u\r\n",COUNT);
			PPRINTF("PB15_count2:%u\r\n",COUNT2);
			PPRINTF("PA4_status:%d\r\n",HAL_GPIO_ReadPin(GPIO_EXTI4_PORT,GPIO_EXTI4_PIN));
		}
		else
		{
			PPRINTF("PB14_status:%d\r\n",HAL_GPIO_ReadPin(GPIO_EXTI14_PORT,GPIO_EXTI14_PIN));
		}
	}
	
  IWDG_Refresh();		
	//+3.3V power sensors	
	if(mode!=3)
	{
		sensor_data->temp1=DS18B20_GetTemp_SkipRom(1);
		if(message==1)
		{
			if((sensor_data->temp1>=-55)&&(sensor_data->temp1<=125))
			{
				PPRINTF("DS18B20_temp1:%.1f\r\n",sensor_data->temp1);
			}
			else
			{
				PPRINTF("DS18B20_temp1:null\r\n");
			}
		}
	}
	
  if((mode==1)||(mode==3))
  {		
		#ifdef USE_SHT
		if(flags==0)
		{
			sensor_data->temp_sht=6553.5;
			sensor_data->hum_sht=6553.5;
		} 
		else if(flags==1)
		{
			float temp2,hum1;
			HAL_I2C_MspInit(&I2cHandle1);
			temp2=SHT20_RT();//get temperature
			hum1=SHT20_RH(); //get humidity
			sensor_data->temp_sht=temp2;
			sensor_data->hum_sht=hum1;
			if(message==1)
			{
				PPRINTF("SHT2x_temp:%.1f,SHT2x_hum:%.1f\r\n",sensor_data->temp_sht,sensor_data->hum_sht);
			}
		}
		else if(flags==2)
		{			
			HAL_I2C_MspInit(&I2cHandle2);			
			tran_SHT31data();		
			sensor_data->temp_sht=sht31_tem;
			sensor_data->hum_sht=sht31_hum;
			if(message==1)
			{
				PPRINTF("SHT3x_temp:%.1f,SHT3x_hum:%.1f\r\n",sensor_data->temp_sht,sensor_data->hum_sht);
			}
		}
		else if(flags==3)
		{		
			bh1750flags=1;		
			I2C_IoInit();
			sensor_data->illuminance=bh1750_read();
			I2C_DoInit();		
			if(message==1)
			{			
				PPRINTF("BH1750_lum:%d\r\n",sensor_data->illuminance);		
			}				
		}		
		#endif
	}
  else if((mode==4)||(mode==9))
  {
		sensor_data->temp2=DS18B20_GetTemp_SkipRom(2);
		if(message==1)
		{			
			if((sensor_data->temp2>=-55)&&(sensor_data->temp2<=125))
			{
				PPRINTF("DS18B20_temp2:%.1f\r\n",sensor_data->temp2);
			}
			else
			{
				PPRINTF("DS18B20_temp2:null\r\n");
			}
		}
		sensor_data->temp3=DS18B20_GetTemp_SkipRom(3);
		if(message==1)
		{
			if((sensor_data->temp3>=-55)&&(sensor_data->temp3<=125))
			{
				PPRINTF("DS18B20_temp3:%.1f\r\n",sensor_data->temp3);
			}
			else
			{
				PPRINTF("DS18B20_temp3:null\r\n");
			}
		}
	}		
	
  //+5V power sensors	
	uint16_t adcdata[3][6];
	
	if(power_time!=0)
	{
		HAL_GPIO_WritePin(PWR_OUT_PORT,PWR_OUT_PIN,GPIO_PIN_RESET);//Enable 5v power supply
		for(uint16_t i=0;i<(uint16_t)(power_time/100);i++)
		{
			 HAL_Delay(100);
       if((i%99==0)&&(i!=0))
			 {
					IWDG_Refresh();		 
			 }				 
		}
	}
	
	if((mode!=3)&&(mode!=8)&&(mode!=9))
	{
		BSP_oil_float_Init();
		for(uint8_t q=0;q<6;q++)
		{
			adcdata[0][q] = HW_AdcReadChannel( ADC_Channel_Oil );//PA0
			HAL_Delay(10);
		}
		HAL_GPIO_WritePin(OIL_CONTROL_PORT,OIL_CONTROL_PIN,GPIO_PIN_SET);
		AD_code1=ADC_Average(adcdata[0]);
		sensor_data->oil=AD_code1*batteryLevel_mV/4095;
		if(message==1)
		{				
			PPRINTF("ADC_PA0:%.3f V\r\n",(sensor_data->oil/1000.0));
		}
	}
	
	if(mode==2)
	{
	  if(mode2_flag==1)
		{
			HAL_I2C_MspInit(&I2cHandle3);		
			LidarLite_init();			 
			sensor_data->distance_mm=LidarLite();	
			if(message==1)
			{				
				PPRINTF("lidar_lite_distance:%d cm\r\n",(sensor_data->distance_mm/10));	
			}				
		}
		else if(mode2_flag==2)
		{
			GPIO_ULT_INPUT_Init();
			GPIO_ULT_OUTPUT_Init();		
			HAL_NVIC_EnableIRQ(TIM3_IRQn);			
			sensor_data->distance_mm=ULT_test();
			HAL_NVIC_DisableIRQ(TIM3_IRQn);
			GPIO_ULT_INPUT_DeInit();
			GPIO_ULT_OUTPUT_DeInit();	
			if(message==1)
			{	
				PPRINTF("ULT_distance:%.1f cm\r\n",(sensor_data->distance_mm)/10.0);
			}
		}
		else if(mode2_flag==3)
	  {
			tfsensor_read_distance(&reading_t);	
		  sensor_data->distance_mm = reading_t.distance_mm;		
			sensor_data->distance_signal_strengh = reading_t.distance_signal_strengh;		
			if(message==1)
			{	
				PPRINTF("TF_distance:%d cm,TF_strength:%d\r\n",(sensor_data->distance_mm/10),sensor_data->distance_signal_strengh);	
			}				
		}
		else
		{
			sensor_data->distance_mm = 65535;		
			sensor_data->distance_signal_strengh = 65535;			
		}			 
	} 
	else if((mode==3)||(mode==8))
	{	
		 BSP_oil_float_Init();
		 for(uint8_t w=0;w<6;w++)
		 {
			 adcdata[0][w] = HW_AdcReadChannel( ADC_Channel_Oil );//PA0			 
			 HAL_Delay(10);				 
		 }
     AD_code1=ADC_Average(adcdata[0]);		 
	   sensor_data->oil=AD_code1*batteryLevel_mV/4095;				 
		 
		 HAL_Delay(50);	
		 for(uint8_t y=0;y<6;y++)
		 {
			 adcdata[1][y] = HW_AdcReadChannel( ADC_Channel_IN1 );//PA1
			 HAL_Delay(10);				 
		 }
     AD_code2=ADC_Average(adcdata[1]);		 
	   sensor_data->ADC_1=AD_code2*batteryLevel_mV/4095;		 
		 
		 HAL_Delay(50);	
		 for(uint8_t z=0;z<6;z++)
		 {
			 adcdata[2][z] = HW_AdcReadChannel( ADC_Channel_IN4 );//PA4	
			 HAL_Delay(10);				 
		 }		 
		 AD_code3=ADC_Average(adcdata[2]);		 
	   sensor_data->ADC_2=AD_code3*batteryLevel_mV/4095;  
		 HAL_GPIO_WritePin(OIL_CONTROL_PORT,OIL_CONTROL_PIN,GPIO_PIN_SET); 	

		 if(message==1)
		 {	
			 PPRINTF("ADC_PA0:%.3f V\r\n",(sensor_data->oil/1000.0));
			 PPRINTF("ADC_PA1:%.3f V\r\n",(sensor_data->ADC_1/1000.0));
			 PPRINTF("ADC_PA4:%.3f V\r\n",(sensor_data->ADC_2/1000.0));
		 }
	}	 
	else if(mode==5)
  {
		WEIGHT_SCK_Init();
		WEIGHT_DOUT_Init();		 
		sensor_data->Weight=Get_Weight();		
		WEIGHT_SCK_DeInit();
		WEIGHT_DOUT_DeInit();			
		if(message==1)
		{	
			PPRINTF("HX711_Weight:%d g\r\n",sensor_data->Weight);
		}
	}	

	GPIO_INPUT_IoInit();
  HAL_Delay(5);	
	sensor_data->in1=HAL_GPIO_ReadPin(GPIO_INPUT_PORT,GPIO_INPUT_PIN1);
	GPIO_INPUT_DeIoInit();
	if(message==1)
	{	
		PPRINTF("PA12_status:%d\r\n",sensor_data->in1);
	}
	
	HAL_GPIO_WritePin(PWR_OUT_PORT,PWR_OUT_PIN,GPIO_PIN_SET);//Disable 5v power supply 
	#endif
	
	if(message==1)
	{	
		HAL_Delay(500);
	}
}

void Device_status( device_t *device_data)
{
	uint8_t freq_band;
	uint16_t version;

	#if defined( REGION_EU868 )
		freq_band=0x01;
	#elif defined( REGION_US915 )
		freq_band=0x02;
	#elif defined( REGION_IN865 )
		freq_band=0x03;
	#elif defined( REGION_AU915 )
		freq_band=0x04;
	#elif defined( REGION_KZ865 )
		freq_band=0x05;
	#elif defined( REGION_RU864 )
		freq_band=0x06;
	#elif defined( REGION_AS923 )
	  #if defined AS923_1
		freq_band=0x08;
	  #elif defined AS923_2
	  freq_band=0x09;
	  #elif defined AS923_3
	  freq_band=0x0A;
	  #elif defined AS923_4
	  freq_band=0x0F;
		#else
		freq_band=0x07;
		#endif
	#elif defined( REGION_CN470 )
		freq_band=0x0B;
	#elif defined( REGION_EU433 )
	  freq_band=0x0C;
	#elif defined( REGION_KR920 )	
	  freq_band=0x0D;
	#elif defined( REGION_MA869 )	
	  freq_band=0x0E;
	#else
    freq_band=0x00;
	#endif
	device_data->freq_band = freq_band;
	
	#if defined( REGION_US915 )	|| defined( REGION_AU915 ) || defined( REGION_CN470 )
	device_data->sub_band = customize_set8channel_get();
	#else
	device_data->sub_band = 0xff;
	#endif
	
	if(fire_version>100)
	{
		version=(fire_version/100)<<8|(fire_version/10%10)<<4|(fire_version%10);
	}
	else
	{
		version=(fire_version/10)<<8|(fire_version%10)<<4;		
	}
	device_data->firm_ver = version;
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
  
  /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx;
  RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2CxCLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

  /*##-2- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  I2Cx_SCL_GPIO_CLK_ENABLE();
  I2Cx_SDA_GPIO_CLK_ENABLE();
  /* Enable I2Cx clock */
  I2Cx_CLK_ENABLE();
  
  /*##-3- Configure peripheral GPIO ##########################################*/  
  /* I2C TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);
    
  /* I2C RX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SDA_PIN;
  GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);
	
}
/**
  * @brief I2C MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  /*##-1- Reset peripherals ##################################################*/
  I2Cx_FORCE_RESET();
  I2Cx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure I2C Tx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
  /* Configure I2C Rx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);
}

void BSP_sensor_Init(void)
{
#if defined(LoRa_Sensor_Node)

//  pwr_control_IoInit();

  if ((mode == 1) || (mode == 3))
  {
#ifdef USE_SHT
		uint8_t txdata1[1] = {0xE7}, txdata2[2] = {0xF3, 0x2D};

		BSP_sht20_Init();

		uint32_t currentTime = TimerGetCurrentTime();
		while (HAL_I2C_Master_Transmit(&I2cHandle1, 0x80, txdata1, 1, 1000) != HAL_OK)
		{
			if (TimerGetElapsedTime(currentTime) >= 500)
			{
				flags = 0;
				break;
			}
		}
		if (HAL_I2C_Master_Transmit(&I2cHandle1, 0x80, txdata1, 1, 1000) == HAL_OK)
		{
			flags = 1;
			PRINTF("\n\rUse Sensor is STH2x\r\n");
		}

		if (flags == 0)
		{
			HAL_I2C_MspDeInit(&I2cHandle1);
			BSP_sht31_Init();

			currentTime = TimerGetCurrentTime();
			while (HAL_I2C_Master_Transmit(&I2cHandle2, 0x88, txdata2, 2, 1000) != HAL_OK)
			{
				if (TimerGetElapsedTime(currentTime) >= 500)
				{
					flags = 0;
					break;
				}
			}
			if (HAL_I2C_Master_Transmit(&I2cHandle2, 0x88, txdata2, 2, 1000) == HAL_OK)
			{
				flags = 2;
				PRINTF("\n\rUse Sensor is STH3x\r\n");
			}
		}

		if (flags == 0)
		{
			float luxtemp;
			HAL_I2C_MspDeInit(&I2cHandle2);
			I2C_IoInit();
			luxtemp = bh1750_read();
			I2C_DoInit();
			if (luxtemp != 0)
			{
				flags = 3;
				PRINTF("\n\rUse Sensor is BH1750\r\n");
			}
		}

		if (flags == 0)
		{
			PRINTF("\n\rNo I2C device detected\r\n");
		}
#endif
  }

  else if (mode == 2)
  {
		uint8_t dataByte[1] = {0x00};
		HAL_GPIO_WritePin(PWR_OUT_PORT, PWR_OUT_PIN, GPIO_PIN_RESET); // Enable 5v power supply
		BSP_lidar_Init();
		waitbusy();
		HAL_I2C_Mem_Write(&I2cHandle3, 0xc4, 0x00, 1, dataByte, 1, 1000);
		if (waitbusy() < 9999)
		{
			mode2_flag = 1;
			PRINTF("\n\rUse Sensor is LIDAR_Lite_v3HP\r\n");
		}
		else
		{
			HAL_I2C_MspDeInit(&I2cHandle3);
			GPIO_ULT_INPUT_Init();
			GPIO_ULT_OUTPUT_Init();
			if (HAL_GPIO_ReadPin(ULT_Echo_PORT, ULT_Echo_PIN) == RESET)
			{
				mode2_flag = 2;
				TIM3_Init();
				PRINTF("\n\rUse Sensor is ultrasonic distance measurement\r\n");
			}
			GPIO_ULT_INPUT_DeInit();
			GPIO_ULT_OUTPUT_DeInit();

			if (mode2_flag == 0)
			{
				if (check_deceive() == 1)
				{
					mode2_flag = 3;
					PRINTF("\n\rUse Sensor is TF-series sensor\r\n");
				}
				else
				{
					PRINTF("\n\rNo distance measurement device detected\r\n");
				}
			}
		}
		HAL_GPIO_WritePin(PWR_OUT_PORT, PWR_OUT_PIN, GPIO_PIN_SET); // Disable 5v power supply
  }
  else if (mode == 5)
  {
		HAL_GPIO_WritePin(PWR_OUT_PORT, PWR_OUT_PIN, GPIO_PIN_RESET); // Enable 5v power supply
		WEIGHT_SCK_Init();
		WEIGHT_DOUT_Init();
		Get_Maopi();
		HAL_Delay(500);
		Get_Maopi();
		WEIGHT_SCK_DeInit();
		WEIGHT_DOUT_DeInit();
		HAL_GPIO_WritePin(PWR_OUT_PORT, PWR_OUT_PIN, GPIO_PIN_SET); // Disable 5v power supply
		PPRINTF("\n\rUse Sensor is HX711\r\n");
  }
  else if ((mode == 7) || (mode == 9))
  {
		GPIO_EXTI15_IoInit(inmode2);
		GPIO_EXTI4_IoInit(inmode3);
		if (mode == 9)
		{
			BSP_oil_float_DeInit();
		}
  }
  else if (mode == 10)
  {
		PPRINTF("\n\r Mode 10 init \r\n");
		BSP_Sensor_Init();
		BSP_Button_Init();
		BSP_Pump_Init();
		BSP_RTC_Init();
  }
//   GPIO_EXTI14_IoInit(inmode);
  GPIO_INPUT_IoInit();

#endif
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
