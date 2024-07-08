/*
 * EngineStateMachine.h
 *
 *  Created on: Feb 24, 2024
 *      Author: win 10
 */

#ifndef ENGINESTATEMACHINE_H_
#define ENGINESTATEMACHINE_H_
#include "main.h"
typedef enum {
    ENGINE_CONTROL_TASK_INIT=0,
    RUNNING_STATE,
    CHECK_DATA_STATE,
    GET_DATA_STATE,
    SIGNAL_STATE,
    CONTROL_SPEED_STATE,
    RECOVERY_STATE
} State;

void EngineControlTaskInit();
void RunningState();
void CheckDataState();
void GetDataState();
void SignalState();
void ControlSpeedState();
void RecoveryState();

#endif /* ENGINESTATEMACHINE_H_ */
