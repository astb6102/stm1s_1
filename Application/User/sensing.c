/*
 * curremt_sensing.c
 *
 *  Created on: Jul 3, 2023
 *      Author: user
 */
#include "stm32f4xx_hal.h"
#include "sensing.h"
#include "CAN.h"
#include "stm32f4xx_hal_adc.h"
#include "PID_controller.h"
#include "FLASH_MEMORY.h"

#define ENCODER_RESOLUTION 8192 // RMB20SC 엔코더의 분해능
#define VOLTAGE_DIV_RATIO   2.56f



const uint16_t ADC_MAX=4095;
const float V_REF=5.28;
uint32_t ENCODER_DATA1;
uint32_t ENCODER_DATA2;
uint16_t ENCODER_VALUE;
int ZERO_POINT_E;
uint32_t ZERO_SETTING_POINT_E;
uint16_t NEW_ZERO;
double CURRENT_POSITION,CURRENT_POSITION_C;
uint16_t PREVIOUS_ENCODER_VALUE;
//double CURRENT_POSITION;
float ZERO_SETTING_POINT_E1=0 ;// 초기 영점 설정 값
int first_start=1000;
float zero_c;
uint16_t ADC_DMA_Buffer_VOLT_HUMID_TEMP[3];
float r997=1500;
#define VREF 3.3
#define VREF_S 5.28
#define ADC_MAX 4095.0
#define R95 2000.0
#define R97 1500.0

float Oil_temp=0;


//ADC_Value Current;
ADC_Value Huminity;
ADC_Value Board_Temperature;
ADC_Value Voltage;
ADC_Value Motor_TEMP;
ADC_Value Actual_V;






void POSITION_sensing()
{

	    ENCODER_DATA1 = ((uint32_t)SPI_DATA[0] & 0x7F) << 8; // 상위 7비트
	    ENCODER_DATA2 = (uint32_t)SPI_DATA[1];               // 하위 8비트
	    ENCODER_VALUE = (ENCODER_DATA1 | ENCODER_DATA2)>>2;     // 두 바이트 병합

	    // 영점 오프셋 계산
	    ZERO_POINT_E = ENCODER_VALUE - ZERO_SETTING_POINT;


	    // 랩어라운드 처리
	    if (ZERO_POINT_E >= 0)
	        NEW_ZERO = (ZERO_POINT_E + 8191) % 8192; // 최대값: 32,768
	    else
	        NEW_ZERO = (8191 + 8192 + ZERO_POINT_E) % 8192;

	    // 현재 위치 계산 (각도 값)
	    CURRENT_POSITION = ((double)(NEW_ZERO/*-16384*/) * (360.0 / 8192.0)); // 각도로 변환

	    // 각도로 변환
	    CURRENT_POSITION = ((double)(NEW_ZERO) * (360.0 / 8192.0));

	    if (CURRENT_POSITION >= 0)
	        ECOMAC_Tx.TCU_SIGN_POSITION = 0x00;
	    else
	        ECOMAC_Tx.TCU_SIGN_POSITION = 0xFF;

	    int16_t scaled_position = (int16_t)(CURRENT_POSITION * 10.0);
	    ECOMAC_Tx.TCU_POSITION_HIGH = (scaled_position >> 8) & 0xFF;
	    ECOMAC_Tx.TCU_POSITION_LOW  = scaled_position & 0xFF;
}




void HUMIDITY_sensing()
{
	Huminity.DF_ADC=Huminity.DF_ADC*0.2f+ADC_DMA_Buffer_VOLT_HUMID_TEMP[0]*0.8f; // 필터
	Huminity.Voltage=Huminity.DF_ADC/ADC_MAX;
	Huminity.Measured_Value=-12.5+(125*Huminity.Voltage);
	if (Huminity.Measured_Value < 0.0f)
		Huminity.Measured_Value = 0.0f;
	else if (Huminity.Measured_Value > 100.0f)
		Huminity.Measured_Value = 100.0f;
	//ECOMAC_Tx.=Huminity.Measured_Value;

}
void BOARD_TEMP_sensing()
{
	Board_Temperature.DF_ADC=Board_Temperature.DF_ADC*0.2f+ADC_DMA_Buffer_VOLT_HUMID_TEMP[1]*0.8f;
	Board_Temperature.Voltage=Board_Temperature.DF_ADC/ADC_MAX;
	Board_Temperature.Measured_Value = -66.875f + 218.75f * Board_Temperature.Voltage;
	//ECOMAC_Tx.Measured_BOARD_TEMP=Board_Temperature.Measured_Value+40;

}
void INPUT_VOLTAGE_sensing()//motor_temp
{
	/*
	Voltage.DF_ADC =Voltage.DF_ADC*0.8f+ADC_DMA_Buffer_VOLT_HUMID_TEMP[2]*0.2f;
	Voltage.Voltage = (Voltage.DF_ADC / ADC_MAX) * VREF;
	Voltage.Cal1 = Voltage.Voltage/(1410);
	Voltage.Cal2=(VREF_S-Voltage.Voltage)/Voltage.Cal1 ;
	//Voltage.Cal1=(Voltage.Voltage)/(R95+R97);
	//Voltage.Cal2=(VREF_S-Voltage.Voltage)/Voltage.Cal1;
	Voltage.Measured_Value=(Voltage.Cal2)*0.40634-376.627;
	ECOMAC_Tx.TCU_Oil_Temp=Voltage.Measured_Value;
	/*Voltage.DF_ADC =Voltage.DF_ADC*0.8f+ADC_DMA_Buffer_VOLT_HUMID_TEMP[2]*0.2f;
	Voltage.Voltage	=(Voltage.DF_ADC*V_REF/ADC_MAX);
	Voltage.Cal1=(Voltage.Voltage)/3500;//전류 계산
	Voltage.Cal2=(V_REF-Voltage.Voltage)/Voltage.Cal1;//RTD 저항 계산

	Voltage.Measured_Value=(Voltage.Cal2- 1000) / 0.385;

	Oil_temp=Voltage.Measured_Value;
		//ECOMAC_Tx.Measured_MOTOR_VOLT=Voltage.Measured_Value;*/
	//Voltage.DF_ADC =Voltage.DF_ADC*0.8f+ADC_DMA_Buffer_VOLT_HUMID_TEMP[2]*0.2f;
	//Voltage.Voltage = (Voltage.DF_ADC / ADC_MAX) * VREF;
	//Voltage.Cal1 = (5.18-Voltage.Voltage)/((Voltage.Voltage)/r997);
	//Voltage.Measured_Value = ((Voltage.Cal1 - 1000.0) / 3.85);
	//ECOMAC_Tx.TCU_Oil_Temp=Voltage.Measured_Value;

	Voltage.DF_ADC =Voltage.DF_ADC*0.8f+ADC_DMA_Buffer_VOLT_HUMID_TEMP[2]*0.2f;
	Voltage.Voltage = (Voltage.DF_ADC / 4096) * VREF;
	Voltage.Cal1 = (1500.0 * 5.18 / Voltage.Voltage) - 1500.0;
	Voltage.Measured_Value = Voltage.Voltage*-334.9678201+1021.789335+40;


	ECOMAC_Tx.TCU_Oil_Temp=Voltage.Measured_Value;
}
