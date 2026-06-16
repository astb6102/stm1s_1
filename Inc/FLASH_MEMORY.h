/*
 * FLASH_MEMORY.h
 *
 *  Created on: Jul 3, 2023
 *      Author: user
 */

#ifndef INC_FLASH_MEMORY_H_
#define INC_FLASH_MEMORY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"


extern uint32_t ZERO_SETTING_POINT;
extern uint16_t ENCODER_VALUE;
extern uint8_t ID;
extern uint8_t EPT_Neutral;
extern uint8_t EPT_STAGE2;
extern int ZERO_SETTING;
extern int ID_SETTING;
extern int EPT_SETTING;
extern uint32_t Adrress_EPT_Neutral;
extern uint32_t Adrress_EPT_STAGTE2;
extern uint32_t ZERO_SETTING_POINT;


void FLASH_MEMORY();
void FLASH_MEMORY_SETTING();

#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH_MEMORY_H_ */
