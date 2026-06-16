/*
 * PID_controller.c
 *
 *  Created on: Jul 3, 2023
 *      Author: user
 */

#include "PID_controller.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <main.h>
#include "mc_interface.h"  // MCI_Handle_t 정의 확인
#include "parameters_conversion.h"  // SDK 매개변수 변환
#include "speed_torq_ctrl.h"
#include "speed_pos_fdbk.h"
#include "mc_type.h"
#include "arm_math.h"
#include "arm_const_structs.h"

// DWT 레지스터 주소 정의
#define DEMCR (*(volatile uint32_t*)0xE000EDFC)       // Debug Exception and Monitor Control Register
#define DWT_CONTROL (*(volatile uint32_t*)0xE0001000) // DWT Control Register
#define DWT_CYCCNT (*(volatile uint32_t*)0xE0001004)  // Cycle Count Register

//extern void Speed_ref_cal(void);
#define POSITION_CMD_MAX 175
#define POSITION_CMD_MIN -175
#define POSITION_DT 0.002
#define MAX_PWM 3599
#define MIN_PWM 0
#define LOAD_THRESHOLD  5
#define MAX_INTEGRAL 10000.0f
#define E_THRESHOLD   0.1f    // 전체 오차 범위의 10%
#define KP_MAX        600.0f
//#define KI_MAX        500.0f
#define KI_MIN        300.0f
#define ANTI_WINDUP_KT 10.0f   // Back-calculation 계수
#define FFT_SIZE 1024
arm_rfft_fast_instance_f32 fft_inst;
int speed_buffer_c=-1;
float anti_phase_accumulator = 0.0f; // 위상 누적기

void fuzzy_pid_adjust(float error, float d_error, float load);

/*---------------variable*/

PID_Control Current_PID;
PID_Control Speed_PID;
PID_Control Position_PID;
double POSITION_ZERO;
double POSITION_CENTER;


float ABS_CURRENT_POSITION;
uint8_t MODE_CONTROL=0;
uint8_t prv_MODE_CONTROL=0;

int RPM_PWM_MAX=0.85;
int RPM_PWM_MIN=0;
int PWM_MAX=3599;
int PWM_MIN=0;

extern int completed;
double CURRENT_TEST;
extern float SHUNT_Current;
float tmpcal1;
float tmpcal2;
int tmp_flag=0;
float cal1=0,cal2=0,cal3=0,cal4=0.9,cal5=0;
float min_ccr=40,max_ccr=170;
int Pre_CMD=1,inposition=1;
int position_cc=0;
float pre_gain=0;
float g1=0.95,g2=0.05;
float pre_Speed_PID=0;
float active_gain=0;
float active_gain_kp=0.2,active_gain_ki=0.7;
float Ramp_cmd=0;
float Ramp_count=0;
float diff_filter0=1;
float diff_filter1=1;
float diff_filter2=1;
float diff_filter3=1;
float Default_torque1,Default_torque2=1; // 토크 설정 syh
float gt;
static uint32_t previous_cycles = 0; // 이전 루프의 사이클 카운트
 uint32_t current_cycles;             // 현재 루프의 사이클 카운트
 float loop_period = 0.0;             // 반복 주기 (초 단위)
 float loop_period_p = 0.0;
 float t1=0;
 float t2=0;
 float t3=0;
 float act_loop_period = 0.0;
 float act_loop_period_p = 0.0;
 float acc_t=3000; //rpm/s 가감속
 float over_D=0;
 float tmp_error=0;
 float actual_load=0;
 float actual_load_filtter=0;
 static float prev_error = 0;
 float Position_target=0;
 float adaptive_postion_gain=1;
 extern float Current_Speed;
 float Output;
 extern float speed_ref_user;
 extern float speed_ref_user1;
 extern float Mode;
 extern int Position_control;
 extern float torque_ref_user;
 extern int operation;
 extern float speed_ref_user_MAX;
 float cal_pos;
 float cc=1;
 int sp_ec=0;
 extern qd_f_t IqdRef_a;
 extern MC_ControlMode_t Current_ControlMode;
 extern float speed_ref_user1;
 extern float Default_torque;
 extern int32_t Default_torque_int;
 extern qd_f_t IqdRef_a;
 float overspeed=0;
 float der_temp;
float outfil1=0,outfil2=0;
float sp_error[200]={0};
float sp_CMD[200]={0};
float mean = 0,mean1=0;
float variance = 0;
float variance1 =0;
float variance2 =0;
int peak_count = 0;
int count_p[200]={0};
int vr_c=0;
int j=0;
int trig=0;
int speed_buffer[FFT_SIZE];
float fft_input[FFT_SIZE];
float fft_output[FFT_SIZE];
float mag_output[FFT_SIZE/2];
float peak_hz;
float freq_resolution =0;
int ready=0;
float anti_signal=0;
float sampling=0;
uint32_t max_idx=0;
float inposition_counter=0;
float dcc_t=0;
 /*------------------*/

void Process_FFT() {
  // 1. ADC 데이터 → float 변환
  for(int o=0; o<FFT_SIZE; o++) {
    fft_input[o] = (speed_buffer[o] * 3.3f) / 4095.0f;  // 12-bit ADC 기준
  }

  // 2. FFT 실행
  arm_rfft_fast_f32(&fft_inst, fft_input, fft_output, 0);

  // 3. 복소수 크기 계산
  arm_cmplx_mag_f32(fft_output, mag_output, FFT_SIZE/2);
}

void FFT_Init() {
  if (arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE) != ARM_MATH_SUCCESS) {
    Error_Handler();
  }
}

void Generate_AntiPhase_Signal(float freq_hz, float amplitude_scale) {
    static float phase = 0.0f;
    float amplitude = speed_ref_user_MAX * ANTI_PHASE_AMPLITUDE * amplitude_scale;

    if(freq_hz > 0.1f) { // 0.1Hz 이상에서만 신호 생성
        float delta_phase = 2 * PI * freq_hz / SAMPLE_RATE;
        phase += delta_phase;

        if(phase > 2*PI) phase -= 2*PI;

        // 180도 위상 지연 신호 생성
       anti_signal = amplitude * arm_sin_f32(phase + PI);

        // 기존 제어 신호와 혼합
       // Default_torque += anti_signal / speed_ref_user_MAX;
    }
}

void Find_Peak_Frequency() {
    float max_mag = 0;
    max_idx = 0;

    // 최대 진폭 검출 (DC 제외)
    for(int p=1; p<FFT_SIZE/2 -1; p++) {
        if(mag_output[p] > max_mag) {
            max_mag = mag_output[p];
            max_idx = p;
        }
    }

    // 이차 보간
    float y1 = mag_output[max_idx -1];
    float y2 = mag_output[max_idx];
    float y3 = mag_output[max_idx +1];
    float delta = (y3 - y1) / (2*(2*y2 - y1 - y3));
    float interpolated_idx = max_idx + delta;

    // 주파수 계산
    freq_resolution = (1/sampling) / FFT_SIZE;
    float peak_freq = interpolated_idx * freq_resolution;
    peak_hz = peak_freq-4;
}







void PID_Init(PID_Control *pid, float Kp, float Ki, float Kd, float error_SUMMAX,float error_SUMMIN, float Dead_Jone)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->error_SUMMAX = error_SUMMAX;
    pid->error_SUMMIN = error_SUMMIN;
    pid->Dead_Jone=Dead_Jone;
}

void POSITION_CONTROLLER()
{

	//POSITION_COUNT=0;
	//cc++;

	cal_pos=CURRENT_POSITION;
	over_D=Position_PID.Dead_Jone/3;
	cal1=Current_PID.CMD-(tmpcal1-0.03);//offset
	cal2=fabs(cal1/Current_PID.CMD);
	actual_load=(tmpcal1 / 0.85);
	actual_load_filtter=actual_load_filtter*0.1+actual_load*0.9;
	overspeed=speed_ref_user_MAX-Current_Speed;
    if(cc==1)
    {
	PID_Init(&Position_PID, 25,0.5,0.5,179.9,-179.9,0.01);
     cc=2;
    }
	if(1)
	{	 // DWT 활성화
		               DEMCR |= 0x01000000;        // DEMCR의 TRCENA 비트를 설정하여 DWT 활성화
		               DWT_CONTROL |= 1;           // DWT의 Cycle Counter 활성화

		               // 현재 사이클 카운트 읽기
		               current_cycles = DWT_CYCCNT;

		               // 반복 주기 계산 (현재 사이클 - 이전 사이클)
		               if (previous_cycles != 0) { // 첫 번째 실행에서는 이전 값이 없으므로 계산 건너뜀
		                   uint32_t cycle_difference = current_cycles - previous_cycles;
		                   loop_period = (float)cycle_difference / SystemCoreClock; // 초 단위로 변환
		                   act_loop_period =act_loop_period+loop_period; // 시간 누적	                   // 디버깅 출력
		               }

		               // 현재 사이클 카운트를 이전 값으로 업데이트
		               previous_cycles = current_cycles;

                       t3=t1-t2;
	}



	//if(cal2>cal4)
   // cal2=1;

	 normalize_angle(&Position_PID.CMD);
	 normalize_angle(&cal_pos);
	// 오차 계산 (목표 위치 - 현재 위치)
    //Position_PID.error =(Position_PID.CMD - CURRENT_POSITION);
	 Position_PID.error = Position_PID.CMD - cal_pos;
	 Position_PID.error = fmod(Position_PID.error + 540.0f, 360.0f) - 180.0f;
/*
    // 오차를 최단 거리로 변환
    if (Position_PID.error > 180.0)
        Position_PID.error -= 360.0; // 반대 방향으로 이동
    else if (Position_PID.error < -180.0)
        Position_PID.error += 360.0; // 반대 방향으로 이동

	 // 목표 각도와 현재 각도를 0~360도로 정규화
	    while (Position_PID.CMD >= 360.0) Position_PID.CMD -= 360.0;
	    while (Position_PID.CMD < 0.0) Position_PID.CMD += 360.0;

	    while (CURRENT_POSITION >= 360.0) CURRENT_POSITION -= 360.0;
	    while (CURRENT_POSITION < 0.0) CURRENT_POSITION += 360.0;
*/
		if(Position_PID.CMD!=Pre_CMD)
			{
				inposition=0;
				position_cc=0;
				if(tmp_flag==0)
				{
				t1=act_loop_period;
				tmp_flag=3;
				}
				Position_target=fabs(Position_PID.error);
				Speed_PID.error_SUM=0;

				if(Mode==1)

					{
					Mode=0;
					MC_StopMotor1();
					}
			}
		 if((Position_PID.Dead_Jone)<fabs(Position_PID.error))
			{
			//inposition=0;
			//operation=1;//stop
			//Mode=0;//torque
			}
		 if((Position_PID.Dead_Jone)>fabs(Position_PID.error))
			{
			 //operation=0;
			// MC_StartMotor1();
			 if(inposition==0)
			 {
			//	 Mode=1;
			// MC_StopMotor1();
			 if(tmp_flag==3)
			 t2=act_loop_period;
			 inposition=1;
			 tmp_flag=0;
			 //IqdRef_a.q =0;
			// IqdRef_a.d =1;
			// MC_SetCurrentReferenceMotor1_F(IqdRef_a);
			// MC_StartMotor1();
			 act_loop_period=0;
			 }
			// IqdRef_a.d =1.0f;
			// IqdRef_a.q =0.0f;
			// Mode=3;



			}

		 else if((Position_PID.Dead_Jone*2)<fabs(Position_PID.error))
			 {

			// IqdRef_a.d =0.0f;
			// IqdRef_a.q =1.0f;
			// MC_StopMotor1();
			// operation=1;
			 if(inposition==1)
			 {
			//	 Mode=0;
			// MC_StopMotor1();
			 inposition=0;
			 }
			// if(Current_ControlMode!=MCM_SPEED_MODE)
			 // MCI_ExecSpeedRamp(&Mci[M1], 0, 0);

			//  Speed_ref_cal();
		    //  Mci[0].pSTC->SpeedRefUnitExt = ((int32_t)speed_ref_user1);
			 //MCI_ExecSpeedRamp(&Mci[M1], 0, 0);

			// IqdRef_a.d =0;
			 }

	       if(speed_buffer_c==-1) ready=0;

	      if(act_loop_period>0.0001)
	      {
	       speed_buffer_c++;

	       speed_buffer[speed_buffer_c]=CURRENT_POSITION;
	       sampling=act_loop_period;
	       act_loop_period=0;
	      }

           if(speed_buffer_c>=FFT_SIZE)
	       {
	    	   speed_buffer_c=-1;
	    	   ready=1;
	       }
           if(ready==1)
               {
               Process_FFT();
               Find_Peak_Frequency();
               float vibration_scale = fminf(mag_output[max_idx] / 1000.0f, 1.0f);
               Generate_AntiPhase_Signal(peak_hz, vibration_scale);

               }

		 //Position_PID.error_SUM = fmaxf(fminf(Position_PID.error_SUM, MAX_INTEGRAL), -MAX_INTEGRAL);
          // max_idx = Find_Peak_Frequency();

//
        if(loop_period>0)
        {
		diff_filter3=(Position_PID.error-Position_PID.PRE_error)/loop_period;
		diff_filter0=diff_filter1*0.5+diff_filter3*0.5;
		Position_PID.error_DER=(diff_filter3/100);
		diff_filter1=diff_filter0;
		tmp_error=Position_PID.error;
		if(Position_PID.error_DER>2000) Position_PID.error_DER=2000;
		if(Position_PID.error_DER<-2000) Position_PID.error_DER=-2000;
		//Position_PID.error_SUM += (Position_PID.error + prev_error) * loop_period ;
		prev_error =Position_PID.error;

			// 적분 한계 설정
		 Position_PID.error_SUM= fmaxf(-MAX_INTEGRAL, fminf(Position_PID.error_SUM, MAX_INTEGRAL));

			// 비선형 보정
		if(fabs(Position_PID.error) < 0.3) {
		Position_PID.error_SUM *= 0.98f; // 미세 오차 시 적분 누적 감소
		}
		float adaptive_gain = 0.6 + 0.5* fabs(Position_PID.error_DER);
		Position_PID.error_SUM += Position_PID.error * loop_period*adaptive_gain;

		//fuzzy_pid_adjust(tmp_error, diff_filter1,actual_load_filtter);
        }



		//if(Position_PID.error_SUM>17)
		//	Position_PID.error_SUM=17;
		//if(Position_PID.error_SUM<-17)
		//	Position_PID.error_SUM=-17;
		if(Position_PID.Dead_Jone>fabs(Position_PID.error))
		{
			//Position_PID.Ki=300;

		}
        if(fabs(Position_PID.error)>50) adaptive_postion_gain=1;
        //if(fabs(Position_PID.error)<30) adaptive_postion_gain=0.5;
       // if(fabs(Position_PID.error)<10) adaptive_postion_gain=0.1;

		Position_PID.P_Control=Position_PID.error*Position_PID.Kp;
		Position_PID.I_Control=Position_PID.error_SUM*Position_PID.Ki;
		Position_PID.D_Control=Position_PID.error_DER*Position_PID.Kd;
		Position_PID.PID_Control=Position_PID.P_Control+Position_PID.I_Control+Position_PID.D_Control*adaptive_postion_gain;
        if(Position_PID.PID_Control>2250) Position_PID.PID_Control=2250;
        if(Position_PID.PID_Control<-2250) Position_PID.PID_Control=-2250;
        Output=Position_PID.PID_Control/2250 * speed_ref_user_MAX;
       // if(Current_Speed>speed_ref_user_MAX)
       // {
      //  Output=Output*-1;

      //  }
        //if(Position_PID.PID_Control>0)
      //  Output=Position_PID.PID_Control/2250+0.2;// * speed_ref_user_MAX;
       // if(Position_PID.PID_Control<0)
       //         Output=Position_PID.PID_Control/2250-0.2;
      //  if(Output>=100 && Output<=300) Output=300;
       // if(Output<=-100 && Output>=-300) Output=-300;

        if(Output<1 && Output>-1) {
        	/*if(Output>=0)
        	IqdRef_a.d =0.2f;
        	if(Output<0)
            IqdRef_a.d =-0.2f;
        	IqdRef_a.q =0.0f;
        	MC_SetCurrentReferenceMotor1_F(IqdRef_a);
        	Mode=-1;*/
        	Output=0;

        }
        else
        {

        	 if(peak_hz>5 && fabs(Position_PID.error)<0.2)
        	        {
        	        //	Output=anti_signal;

        	        }

        }

       // if(Output>0 && Output<0.5) Output=0.5;
       // if(Output<0 && Output>-0.5) Output=-0.5;
        speed_ref_user=Output*-1;
        //IqdRef_a.q =Output*-1;
   	 if(Current_ControlMode!=MCM_SPEED_MODE)
	  MCI_ExecSpeedRamp(&Mci[M1], 0, 0);
	 speed_ref_user1=((int32_t)((float)speed_ref_user/3))<<15;
	 Mci[0].pSTC->SpeedRefUnitExt = ((int32_t)speed_ref_user1);

	 if(peak_hz>5 && fabs(Position_PID.error)<0.2)
	 {
	    MC_SetCurrentReferenceMotor1_F(IqdRef_a);
       //  Default_torque = Output*-1/speed_ref_user_MAX;
        //Default_torque_int=Default_torque*32767;
        if(Default_torque>0)
        {
        PIDIqHandle_M1.hUpperOutputLimit=Default_torque_int;
       PIDIqHandle_M1.hLowerOutputLimit=Default_torque_int;
        IqdRef_a.d =(1-overspeed/speed_ref_user_MAX);
        //IqdRef_a.q = 1.5;

       }

        if(Default_torque<0)
		{
		PIDIqHandle_M1.hUpperOutputLimit=-Default_torque_int;
		PIDIqHandle_M1.hLowerOutputLimit=-Default_torque_int;
		IqdRef_a.d =(1-overspeed/speed_ref_user_MAX)*-1;
		//IqdRef_a.q = -1.5;
		}
	 }

         //Speed_PID.CMD=Output*-1;
         //SPEED_CONTROLLER();


        //operation=1;
        Pre_CMD=Position_PID.CMD;
		pre_gain=Output;

	}
void POSITION_CONTROLLER_SPEED()
{

	//POSITION_COUNT=0;
	//POSITION_CAL();

	cal_pos=CURRENT_POSITION;
	over_D=Position_PID.Dead_Jone/3;
	cal1=Current_PID.CMD-(tmpcal1-0.03);//offset
	cal2=fabs(cal1/Current_PID.CMD);
	actual_load=(tmpcal1 / 0.85);
	actual_load_filtter=actual_load_filtter*0.1+actual_load*0.9;
	if(cc==1)
	{
	PID_Init(&Position_PID, 1500,0,0,179.9,-179.9,0.05);
    cc=3;
	}
	if(1)
	{	 // DWT 활성화
		               DEMCR |= 0x01000000;        // DEMCR의 TRCENA 비트를 설정하여 DWT 활성화
		               DWT_CONTROL |= 1;           // DWT의 Cycle Counter 활성화

		               // 현재 사이클 카운트 읽기
		               current_cycles = DWT_CYCCNT;

		               // 반복 주기 계산 (현재 사이클 - 이전 사이클)
		               if (previous_cycles != 0) { // 첫 번째 실행에서는 이전 값이 없으므로 계산 건너뜀
		                   uint32_t cycle_difference = current_cycles - previous_cycles;
		                   loop_period_p = (float)cycle_difference / SystemCoreClock; // 초 단위로 변환
		                   act_loop_period_p =act_loop_period_p+loop_period_p; // 시간 누적	                   // 디버깅 출력
		               }

		               // 현재 사이클 카운트를 이전 값으로 업데이트
		               previous_cycles = current_cycles;

                       t3=t1-t2;
	}



	//if(cal2>cal4)
   // cal2=1;

	 normalize_angle(&Position_PID.CMD);
	 normalize_angle(&cal_pos);
	// 오차 계산 (목표 위치 - 현재 위치)
    //Position_PID.error =(Position_PID.CMD - CURRENT_POSITION);
	 Position_PID.error = Position_PID.CMD - cal_pos;
	 Position_PID.error = fmod(Position_PID.error + 540.0f, 360.0f) - 180.0f;
/*
    // 오차를 최단 거리로 변환
    if (Position_PID.error > 180.0)
        Position_PID.error -= 360.0; // 반대 방향으로 이동
    else if (Position_PID.error < -180.0)
        Position_PID.error += 360.0; // 반대 방향으로 이동

	 // 목표 각도와 현재 각도를 0~360도로 정규화
	    while (Position_PID.CMD >= 360.0) Position_PID.CMD -= 360.0;
	    while (Position_PID.CMD < 0.0) Position_PID.CMD += 360.0;

	    while (CURRENT_POSITION >= 360.0) CURRENT_POSITION -= 360.0;
	    while (CURRENT_POSITION < 0.0) CURRENT_POSITION += 360.0;
*/
		if(Position_PID.CMD!=Pre_CMD)
			{
				inposition=0;
				position_cc=0;
				if(tmp_flag==0)
				{
				t1=act_loop_period_p;
				tmp_flag=3;
				}
				Position_target=fabs(Position_PID.error);
				Speed_PID.error_SUM=0;

				if(Mode==1)

					{
					Mode=0;
					MC_StopMotor1();
					}
			}
		 if((Position_PID.Dead_Jone)<fabs(Position_PID.error))
			{
			//inposition=0;
			//operation=1;//stop
			//Mode=0;//torque
			}
		 if((Position_PID.Dead_Jone)>fabs(Position_PID.error))
			{
			 //operation=0;
			// MC_StartMotor1();
			Position_PID.error=0;
			 if(inposition==0)
			 {
			//	 Mode=1;
			// MC_StopMotor1();
			 if(tmp_flag==3)
			 t2=act_loop_period_p;
			 inposition=1;
			 tmp_flag=0;
			 //IqdRef_a.q =0;
			// IqdRef_a.d =1;
			// MC_SetCurrentReferenceMotor1_F(IqdRef_a);
			// MC_StartMotor1();
			// act_loop_period=0;
			 }
			// IqdRef_a.d =1.0f;
			// IqdRef_a.q =0.0f;
			// Mode=3;



			}

		 else if((Position_PID.Dead_Jone)<fabs(Position_PID.error))
			 {

			// IqdRef_a.d =0.0f;
			// IqdRef_a.q =1.0f;
			// MC_StopMotor1();
			// operation=1;
			 if(inposition==1)
			 {
			//	 Mode=0;
			// MC_StopMotor1();
			 inposition=0;
			 }
			// if(Current_ControlMode!=MCM_SPEED_MODE)
			 // MCI_ExecSpeedRamp(&Mci[M1], 0, 0);

			//  Speed_ref_cal();
		    //  Mci[0].pSTC->SpeedRefUnitExt = ((int32_t)speed_ref_user1);
			 //MCI_ExecSpeedRamp(&Mci[M1], 0, 0);

			// IqdRef_a.d =0;
			 }

//
		 //Position_PID.error_SUM = fmaxf(fminf(Position_PID.error_SUM, MAX_INTEGRAL), -MAX_INTEGRAL);

//
        if(loop_period_p>0)
        {
		diff_filter3=(Position_PID.error-Position_PID.PRE_error)/loop_period_p;
		diff_filter0=diff_filter1*0.2+diff_filter3*0.8;
		Position_PID.error_DER=(diff_filter0/1000);
		diff_filter1=diff_filter0;
		tmp_error=Position_PID.error;
		if(Position_PID.error_DER>2000) Position_PID.error_DER=2000;
		if(Position_PID.error_DER<-2000) Position_PID.error_DER=-2000;
		//Position_PID.error_SUM += (Position_PID.error + prev_error) * loop_period ;
		prev_error =Position_PID.error;

			// 적분 한계 설정
		// Position_PID.error_SUM= fmaxf(-MAX_INTEGRAL, fminf(Position_PID.error_SUM, MAX_INTEGRAL));

			// 비선형 보정
		//if(fabs(Position_PID.error) < 0.2) {
		//Position_PID.error_SUM *= 0.99f; // 미세 오차 시 적분 누적 감소
	//	}
		float adaptive_gain = 1 + 0.5* fabs(Position_PID.error_DER);
		Position_PID.error_SUM += Position_PID.error * loop_period_p;//*adaptive_gain;

		//fuzzy_pid_adjust(tmp_error, diff_filter1,actual_load_filtter);
        }



		if(Position_PID.error_SUM>1000)
			Position_PID.error_SUM=1000;
		if(Position_PID.error_SUM<-1000)
			Position_PID.error_SUM=-1000;
		if(Position_PID.Dead_Jone>fabs(Position_PID.error))
		{
			//Position_PID.Ki=300;

		}
        if(fabs(Position_PID.error)>50) adaptive_postion_gain=1;
        //if(fabs(Position_PID.error)<30) adaptive_postion_gain=0.5;
       // if(fabs(Position_PID.error)<10) adaptive_postion_gain=0.1;

		Position_PID.P_Control=Position_PID.error*Position_PID.Kp;
		Position_PID.I_Control=Position_PID.error_SUM*Position_PID.Ki;
		Position_PID.D_Control=Position_PID.error_DER*Position_PID.Kd;
		Position_PID.PID_Control=Position_PID.P_Control+Position_PID.I_Control+Position_PID.D_Control*adaptive_postion_gain;
        if(Position_PID.PID_Control>2250) Position_PID.PID_Control=2250;
        if(Position_PID.PID_Control<-2250) Position_PID.PID_Control=-2250;
        Output=Position_PID.PID_Control/2250 * speed_ref_user_MAX;
        //if(Position_PID.PID_Control>0)
      //  Output=Position_PID.PID_Control/2250+0.2;// * speed_ref_user_MAX;
       // if(Position_PID.PID_Control<0)
       //         Output=Position_PID.PID_Control/2250-0.2;
       // if(Output>=100 && Output<=400) Output=400;
       // if(Output<=-100 && Output>=-400) Output=-400;
   /*
        if(Position_target>15 && fabs(Position_PID.error)<15 && fabs(Position_PID.error)>5)
        	dcc_t=fabs(Position_PID.error)/15;
        else dcc_t=1;

        if(Position_target>5 && (Position_target-fabs(Position_PID.error))<5 && (Position_target-fabs(Position_PID.error))>0.5)
             	dcc_t=(Position_target-fabs(Position_PID.error))/5;
             else dcc_t=1;
*/

        if(Output<1 && Output>-1)  {
        	/*if(Output>=0)
        	IqdRef_a.d =0.2f;
        	if(Output<0)
            IqdRef_a.d =-0.2f;
        	//IqdRef_a.q =0.0f;
        	MC_SetCurrentReferenceMotor1_F(IqdRef_a);
        	//Mode=-1;*/
        	Output=0;

        }
        else
        {
        	// if(Current_ControlMode!=MCM_SPEED_MODE)
		 // MCI_ExecSpeedRamp(&Mci[M1], 0, 0);
        	//IqdRef_a.d =0.0f; test
        	//MC_SetCurrentReferenceMotor1_F(IqdRef_a);
       // Speed_PID.CMD=Output*-1;
        }


        //speed_ref_user=Output*-1;
       // speed_ref_user1=((int32_t)((float)speed_ref_user/3))<<15;
	   // Mci[0].pSTC->SpeedRefUnitExt = ((int32_t)speed_ref_user1);
      //  outfil1=outfil2*0.95+Output*0.05;
     //   outfil2=outfil1;
        if(0.1>fabs(Position_PID.error))
        {
        	//IqdRef_a.d=0.2;
        	//Output=0;

        }
        else
        {
        	//IqdRef_a.d=0.0;

        }

        Speed_PID.CMD=Output*-1;//*dcc_t;
         //SPEED_CONTROLLER();
         Mode=5;

        //operation=1;
        Pre_CMD=Position_PID.CMD;
		pre_gain=Output;

	}
// [1] 각도 정규화 모듈
void normalize_angle(float *angle) {
    *angle = fmod(*angle, 360.0f);
    if (*angle < 0) *angle += 360.0f;
}

// [2] 퍼지 PID 게인 조정기 (신규 추가)
void fuzzy_pid_adjust(float error, float d_error, float load) {
    // 부하 가중치 적용
    float load_gain = load+0.7;
    float position_gain = (1-fabs((Position_PID.error/180))) + 0.1;
    if(load_gain>1.7) load_gain=1.7;
    if(load_gain<0.5) load_gain=0.5;

    //Position_PID.Kd=1.4*load_gain*position_gain;
    if(position_gain>1) position_gain=1;
   // Position_PID.Ki=0.5*fabs(position_gain);
    if(fabs(Position_PID.error)<20 && inposition==0)
    {
    	//Position_PID.Kd=1.2;

	//	Position_PID.Ki=0.5;
    }
    else if(fabs(Position_PID.error)<0.3 && inposition==1)
    {
    	//Position_PID.Kd=2;
   //     Position_PID.Ki=1;
    }

    /*// 3차원 퍼지 규칙 테이블
    if(fabs(d_error) > 40.0f * load_gain) {
        Position_PID.Kd = 2.0f * load_gain;  // 부하 비례 Kd 증가
    }
    else if(fabs(d_error) < 30.0f * load_gain) {
        Position_PID.Kd = 1.5f * load_gain;
    }
    else {
        Position_PID.Kd = 0.8f * load_gain;
    }*/
}








void SPEED_CONTROLLER()
{
    //SPEED_COUNT = 0;

    // SPEED_COUNT = 0;
 int i;
           // DWT 활성화

               DEMCR |= 0x01000000;        // DEMCR의 TRCENA 비트를 설정하여 DWT 활성화
               DWT_CONTROL |= 1;           // DWT의 Cycle Counter 활성화

               // 현재 사이클 카운트 읽기
               current_cycles = DWT_CYCCNT;

               // 반복 주기 계산 (현재 사이클 - 이전 사이클)
               if (previous_cycles != 0) { // 첫 번째 실행에서는 이전 값이 없으므로 계산 건너뜀
                   uint32_t cycle_difference = current_cycles - previous_cycles;
                   loop_period = (float)cycle_difference / SystemCoreClock; // 초 단위로 변환
                   act_loop_period =act_loop_period+loop_period; // 시간 누적

               }

               // 현재 사이클 카운트를 이전 값으로 업데이트
               previous_cycles = current_cycles;
              if(speed_buffer_c==-1) ready=0;

              speed_buffer_c++;

       speed_buffer[speed_buffer_c]=Current_Speed;
       if(speed_buffer_c>=1024)
       {
    	   speed_buffer_c=-1;
    	   ready=1;
       }
        // 절대값 계산 및 오차 계산
        Speed_PID.ABS_CMD = fabs(Speed_PID.CMD);
        Speed_PID.error = Speed_PID.CMD - Current_Speed;

        /*if(vr_c>2000)
        	Speed_PID.error=0;*/

        // 데드 존 처리
        if (Speed_PID.CMD == 0)
        {
            Speed_PID.error = 0;
            Speed_PID.error_SUM=0;
            Speed_PID.error_DER=0;
        }
        // 적분 항 계산 (Anti-Windup 처리 포함)
        if (fabs(Speed_PID.PID_Control)>= speed_ref_user_MAX)
        {
            if(Speed_PID.PID_Control>0) Speed_PID.PID_Control= speed_ref_user_MAX; // Windup 방지
            if(Speed_PID.PID_Control<0) Speed_PID.PID_Control= -speed_ref_user_MAX;
        }

         Speed_PID.error_SUM += Speed_PID.error *loop_period;
         Speed_PID.error_SUMMAX= speed_ref_user_MAX/Speed_PID.Ki;
         Speed_PID.error_SUM=fmaxf(-Speed_PID.error_SUMMAX, fminf(Speed_PID.error_SUM, Speed_PID.error_SUMMAX));
         Speed_PID.error_SUMMIN=speed_ref_user_MAX/Speed_PID.Kd;


        if(loop_period>0)
        // 미분 항 계산
        {
        der_temp=(Speed_PID.error - Speed_PID.PRE_error) / loop_period;

        Speed_PID.error_DER = Speed_PID.error_DER*0.2+der_temp*0.8;
        Speed_PID.error_DER=fmaxf(-1000, fminf(Speed_PID.error_DER, 1000));

        // PID 제어 값 계산
        Speed_PID.P_Control = Speed_PID.Kp * Speed_PID.error;
        Speed_PID.I_Control = Speed_PID.Ki * Speed_PID.error_SUM;
        Speed_PID.D_Control = Speed_PID.Kd * Speed_PID.error_DER;
        Speed_PID.PID_Control = Speed_PID.P_Control + Speed_PID.I_Control + Speed_PID.D_Control;
        }
        if (fabs(Speed_PID.PID_Control)>= speed_ref_user_MAX)
		  {
			  if(Speed_PID.PID_Control>0) Speed_PID.PID_Control= speed_ref_user_MAX; // Windup 방지
			  if(Speed_PID.PID_Control<0) Speed_PID.PID_Control= -speed_ref_user_MAX;
		  }
        // 가감속 제어
        // 최대 가속도 및 감속도 설정
        const float MAX_ACCELERATION = 0.1; // rpm/s
        const float MAX_DECELERATION = -0.1; // rpm/s

        // 현재 속도와 목표 속도 간의 차이를 계산하여 가속도/감속도를 결정
        float acceleration = 0;
        if (Speed_PID.error > 0) // 목표 속도보다 낮을 때 가속
        {
            acceleration = MAX_ACCELERATION;
        }
        else if (Speed_PID.error < 0) // 목표 속도보다 높을 때 감속
        {
            acceleration = MAX_DECELERATION;
        }

       

        // 이전 오차 저장
        Speed_PID.PRE_error = Speed_PID.error;
   /*
        j++;
                if(j>10)
                {
                sp_error[sp_ec]=Speed_PID.error;
                sp_CMD[sp_ec]=Speed_PID.CMD;
                count_p[sp_ec]=peak_count;
                sp_ec++;
                if(sp_ec>200) sp_ec=0;
                j=0;


        for(int i=0; i<200; i++)
        {
        mean += sp_error[i];
        mean1 +=sp_CMD[i] ;
        }
        mean /= 200;
        mean1/=200;
        for(int i=0; i<200; i++)
        {
            variance += (sp_error[i] - mean) * (sp_error[i] - mean);
        }
        variance /= 200;
        variance1=variance*0.001+variance2*0.999;
        variance2=variance1;

        for(int i=1; i<199; i++)
        {
            if( (sp_error[i] > sp_error[i-1] && sp_error[i] > sp_error[i+1]) ||
                (sp_error[i] < sp_error[i-1] && sp_error[i] < sp_error[i+1]) )
            {
                peak_count++;
            }
        }
                }
       if(variance1<500) peak_count=0;
       if(count_p[0]==peak_count && count_p[100]==peak_count && count_p[199]==peak_count)
       {
    	   trig++;
    	   if(trig>100)
    	   {
    		    peak_count=0;
    	   }
         }
       else
         	   {
         		   trig=0;
         	   }
      */


        /*
        if(peak_count>300)
        {
        	Speed_PID.PID_Control*=0.95;
        	Speed_PID.error *=0.95;
			IqdRef_a.d+=0.0001;
			if(IqdRef_a.d>2) IqdRef_a.d=2;
        IqdRef_a.q *=0.9999;
         if(IqdRef_a.q<0.5) IqdRef_a.q=0.5;
        	MC_SetCurrentReferenceMotor1_F(IqdRef_a);

        	//peak_count=0;
        }
        else
        {
        Speed_PID.PID_Control = pre_Speed_PID*0.4+Speed_PID.PID_Control*0.6;
        IqdRef_a.d *=0.9999;
         IqdRef_a.q *=1.00001;
         if(IqdRef_a.q>1.5) IqdRef_a.q=1.5;
         MC_SetCurrentReferenceMotor1_F(IqdRef_a);
        //if(IqdRef_a.q>1.5) IqdRef_a.q=1.5;
        }
         */
        //fft
        if(speed_buffer_c==-1) ready=0;

        	      if(act_loop_period>0.0001)
        	      {
        	       speed_buffer_c++;

        	       speed_buffer[speed_buffer_c]=CURRENT_POSITION;
        	       sampling=act_loop_period;
        	       act_loop_period=0;
        	      }

                   if(speed_buffer_c>=FFT_SIZE)
        	       {
        	    	   speed_buffer_c=-1;
        	    	   ready=1;
        	       }
                   if(ready==1)
                       {
                       Process_FFT();
                       Find_Peak_Frequency();
                       float vibration_scale = fminf(mag_output[max_idx] / 1000.0f, 1.0f);
					  Generate_AntiPhase_Signal(peak_hz, vibration_scale);
                       }

        		 //Position_PID.error_SUM = fmaxf(fminf(Position_PID.error_SUM, MAX_INTEGRAL), -MAX_INTEGRAL);
                  // max_idx = Find_Peak_Frequency();

                  if(fabs(anti_signal)>50 && fabs(Position_PID.error)<0.2 && Position_control==2 )
                      {

                	 Speed_PID.PID_Control=Speed_PID.PID_Control*0.3+(-anti_signal);
                      }

        if(fabs(Position_PID.error)<0.3)
        		{
        	inposition_counter=inposition_counter+loop_period;
        	if(inposition_counter>100) inposition_counter=100;
        		}
        else
        	{
        	inposition_counter=0;
        	}
        if(inposition_counter<0.2||Position_control==0)
        {
        	  Default_torque1= Speed_PID.PID_Control/speed_ref_user_MAX;
              Default_torque=Default_torque1*Default_torque2;
              if(fabs(Default_torque1)<0.005) Default_torque=0;



              Default_torque_int=Default_torque*32767;

              if(Default_torque>0)
                 {
                 PIDIqHandle_M1.hUpperOutputLimit=Default_torque_int;
                PIDIqHandle_M1.hLowerOutputLimit=Default_torque_int;
               //  IqdRef_a.d =(1-overspeed/speed_ref_user_MAX);
                 //IqdRef_a.q = 1.5;

                }

                 if(Default_torque<0)
         		{
         		PIDIqHandle_M1.hUpperOutputLimit=-Default_torque_int;
         		PIDIqHandle_M1.hLowerOutputLimit=-Default_torque_int;
         		//IqdRef_a.d =(1-overspeed/speed_ref_user_MAX)*-1;
         		//IqdRef_a.q = -1.5;
         		}
             // PIDIqHandle_M1.hUpperOutputLimit=Default_torque_int;
      	    //PIDIqHandle_M1.hLowerOutputLimit=-Default_torque_int;
        }
    	pre_Speed_PID=Speed_PID.PID_Control;



}


