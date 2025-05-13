#ifndef APPL_TIMER_H
#define APPL_TIMER_H
#include "stdint.h"
typedef struct
{
  uint32_t u32PrevMills;
  uint32_t u32CurrMills;
  uint32_t u32TargetMills;
  uint8_t bEnable;
}stcTimer;

typedef enum
{
  enTimerNormalStop = 0,
  enTimerAutoRestart,
}TimerMode;

void InitilizeTimer(stcTimer *pTimer);
void StartTimer(stcTimer *pTimer , uint32_t DelayTimeMs);
uint8_t Timer_IsTimeout(stcTimer *pTimer , TimerMode m_TimerMode);
void StopTimer(stcTimer *pTimer);
uint8_t Timer_IsRunning(stcTimer *pTimer);

#endif
