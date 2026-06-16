/*
 * PID_controller.h
 *
 *  Created on: Jul 3, 2023
 *      Author: user
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#define ANTI_PHASE_AMPLITUDE 0.3f // 역상 신호 최대 진폭 (전체 출력 대비 비율)
#define SAMPLE_RATE 10000.0f      // 샘플링 주파수 (Hz)


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "sensing.h"

typedef struct
{
	float CMD;
	float ABS_CMD;
	float Measured_Value;
	float error;
	float error_SUM;
	float error_SUMMAX;
	float error_SUMMIN;
	float error_DER;
	float PRE_error;
	float Kp;
	float Ki;
	float Kd;
	float P_Control;
	float I_Control;
	float D_Control;
	float PID_Control;
	float Dead_Jone;


}PID_Control;

extern PID_Control Current_PID;
extern PID_Control Speed_PID;
extern PID_Control Position_PID;
//extern int ZERO_SETTING_POINT;

extern uint8_t MOTOR_CW_CCW;
extern int ZERO_POINT;
extern uint16_t ENCODER_VALUE;
extern uint32_t MOTOR_MEASUREMENT_RPM_M;
extern float MOTOR_MEASUREMENT_RPM_T;
extern float SHUNT_Voltage;
extern float SHUNT_Current;
extern const uint16_t ADC_MAX;
extern int MOTOR_IN_PWM_CCR;
extern double CURRENT_POSITION;
extern uint16_t SPEED_COUNT;
extern uint16_t POSITION_COUNT;
extern int tmp_flag,tmp_flag1;
extern uint8_t MODE_CONTROL;
extern uint8_t prv_MODE_CONTROL;
extern float cal1,cal2,cal3,cal4,cal5;
extern float min_ccr, max_ccr;
extern int Pre_CMD,inposition;
extern float tmpcal1;
extern float pre_gain;

//void PID_Init(PID_Control *pid, float Kp, float Ki, float Kd, float error_SUMMAX);

void normalize_angle(float *angle);
void POSITION_CONTROLLER_SPEED(void);

void POSITION_CONTROLLER();
void SPEED_CONTROLLER();
void CURRENT_CONTROLLER();
void PID_Init(PID_Control *pid, float Kp, float Ki, float Kd, float error_SUMMAX, float error_SUMMIN, float Dead_Jone);
#ifdef __cplusplus
}
#endif

#endif /* INC_PID_CONTROLLER_H_ */
