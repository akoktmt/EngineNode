/*
 * EngineStateMachine.c
 *
 *  Created on: Feb 24, 2024
 *      Author: win 10
 */
#include "main.h"
#include "EngineStateMachine.h"
#include <stdio.h>
#include "Motor_Driver.h"
#include <string.h>
#include "EncoderVelocity.h"
#include "PID.h"
#include "CAN_Handle.h"
#include "tim.h"
#include "usart.h"

extern   double VelSetpoint;
extern  Encoder Enco;
extern  double speed;
extern  double PIDOut,PWM;
extern  PID_TypeDef TPID;
extern  VNH3SP30_t driver;
extern uint32_t Time;
extern State currentState;
extern uint8_t rcdata[8];
extern uint8_t Can_RecFlag;
void EngineControlTaskInit() {
		Time=1;
		VelSetpoint = 0;
		VNH3SP30_Init(&driver);
		Encoder_Init(&Enco);
		PID(&TPID, &speed, &PIDOut, &VelSetpoint, 4, 0, 3, _PID_P_ON_E, _PID_CD_DIRECT);
		PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
		PID_SetSampleTime(&TPID, Time);
		PID_SetOutputLimits(&TPID, -1, 1);
		currentState=RUNNING_STATE;
}

void RunningState() {
	while(Can_RecFlag!=1);
	currentState=GET_DATA_STATE;
}
void GetDataState() {
	 TPID.MySetpoint=rcdata[2];
	 Encoder_CaculateSpeed(&Enco,Time);
	 speed=Get_Speed(&Enco);
	 currentState=CONTROL_SPEED_STATE;
}
void ControlSpeedState() {
	PWM += PIDOut;
	VNH3SP30_SetSpeed(&driver,PWM);
	PID_Compute(&TPID);
	currentState=GET_DATA_STATE;
}

void RecoveryState() {
	currentState=RUNNING_STATE;
}
