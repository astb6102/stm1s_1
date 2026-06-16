/*
 * current_sensing.h
 *
 *  Created on: Jul 3, 2023
 *      Author: user
 */

#ifndef INC_SENSING_H_
#define INC_SENSING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <math.h>

typedef struct
{
	uint16_t ADC_Value;
	float DF_ADC;
	float Voltage;
	float DF_Voltage;
	float Cal1;
	float Cal2;
	float Cal3;
	float Resistance;
	float Measured_Value;

}ADC_Value;

extern ADC_Value Current;
extern ADC_Value Huminity;
extern ADC_Value Temperature;
extern ADC_Value Voltage;
extern ADC_Value Motor_TEMP;
extern ADC_Value Actual_V;
extern uint8_t SPI_DATA[2];
extern uint16_t ADC_DMA_Buffer_VOLT_HUMID_TEMP[3];

void TEMP();
//void current_sensing();
//void VOLTAGE_sensing();
void HUMIDITY_sensing();
//void FET_TEMP_sensing();
//void MOTOR_TEMP_SENSING();
//void Actual_V_sensing();
void POSITION_sensing();
void BOARD_TEMP_sensing();
void INPUT_VOLTAGE_sensing();//motor_temp

#ifdef __cplusplus
}
#endif




#endif /* INC_SENSING_H_ */
