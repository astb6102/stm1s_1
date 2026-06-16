/*
 * CAN.h
 *
 *  Created on: Jun 20, 2024
 *      Author: user
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef struct
{
    uint8_t VCU_RDY;
	uint8_t VCU_ShiftCMD;
	uint8_t VCU_EPT_Control;
	uint8_t VCU_SIGN_POSITION;
	uint8_t VCU_POSITION_HIGH;
	uint8_t VCU_POSITION_LOW;
	int16_t VCU_POSITION;
	uint8_t CAN_EPT_Neutral;
	uint8_t CAN_EPT_STAGE2;
	uint8_t VCU_ZERO_SET;
	uint8_t EPT_SET;
    uint8_t	Reserved1;
} ECOMAC;

typedef struct
{
	uint8_t TCU_DATA;
	uint8_t TCU_Oil_Temp;
	uint8_t Reserved1;
	uint8_t Reserved2;
	uint8_t TCU_Fault_Code;
	uint8_t TCU_SIGN_POSITION;
	uint8_t TCU_POSITION_HIGH;
	uint8_t TCU_POSITION_LOW;

} ECOMAC_DATA;

extern ECOMAC ECOMAC_Rx;
extern ECOMAC_DATA ECOMAC_Tx;

extern CAN_FilterTypeDef CAN2_FILTER;
extern CAN_RxHeaderTypeDef CAN2_RxHeader;
extern CAN_TxHeaderTypeDef CAN2_TxHeader;
extern uint8_t CAN2_Rx0Data[8];
extern uint32_t TxMailBox;
extern uint8_t CAN2_Tx0Data[8];
extern volatile uint8_t CAN2_Rx_Flag;
extern uint8_t System_Reset;
extern uint8_t ResponseMode;



extern float Neutral;
extern int Stage1;
extern float Stage2;

extern int CPU_ERROR;
extern int SPI_ERROR;
extern int V_LEVEL_ERROR;
extern int CAN_ERROR;
extern int ADC_ERROR;
extern int TCU_Ready;

extern double CMD_POSITION;
extern double ERROR_POSITION;

extern uint8_t CBIT_TEST1;
extern uint8_t CBIT_TEST2;
extern uint8_t CBIT_TEST3;
extern uint8_t CBIT_TEST4;
extern uint8_t CBIT_TEST5;
extern uint8_t CBIT_TEST6;

void Not_Runing_MSG_SET();
void Runnig_MSG_SET();
void Completed_MSG_SET();
void CMD_Operation();
void VCU_RDY();
void CAN_Packet_Build();
void CAN_SET();
void parse_CAN_Data();
void ECOMAC_Tx_Data_Build();
void Update_Control_Variable();
void Check_Position_Command_Completion();
void Update_TCU_ShiftOpSts();
#endif /* INC_CAN_H_ */
