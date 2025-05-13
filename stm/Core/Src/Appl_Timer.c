
#include "Appl_Timer.h"
#include <stdbool.h>

#include "stdlib.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

void InitilizeTimer(stcTimer *pTimer)
{
	pTimer->u32PrevMills = 0u;
	pTimer->u32CurrMills = 0u;
	pTimer->u32TargetMills = 0u;
	pTimer->bEnable = false;
}


void StartTimer(stcTimer *pTimer , uint32_t DelayTimeMs)
{
	if(false == Timer_IsRunning(pTimer))
	{
		InitilizeTimer(pTimer);
		pTimer->u32TargetMills = DelayTimeMs;
		pTimer->u32PrevMills = HAL_GetTick();
		pTimer->bEnable = true;
	}
}

//void StartTimer(stcTimer *pTimer , uint32_t DelayTimeMs)
//{
//    SendMsg_RPI((uint8_t*)"Debug: StartTimer Called\n", strlen("Debug: StartTimer Called\n"));
//
//    if(false == Timer_IsRunning(pTimer))
//    {
//        pTimer->u32TargetMills = DelayTimeMs;
//
//        // Ensure that `u32PrevMills` is set to a valid value
//        pTimer->u32PrevMills = HAL_GetTick();
//        if (pTimer->u32PrevMills == 0 || pTimer->u32PrevMills > HAL_GetTick()) {
//            SendMsg_RPI((uint8_t*)"Error: Invalid u32PrevMills, resetting to HAL_GetTick()\n",
//                        strlen("Error: Invalid u32PrevMills, resetting to HAL_GetTick()\n"));
//            pTimer->u32PrevMills = HAL_GetTick();
//        }
//
//        pTimer->bEnable = true;
//
//        char debugBuffer[64];
//        snprintf(debugBuffer, sizeof(debugBuffer), "Debug: Timer Started for %lu ms (PrevMills: %lu)\n",
//                 DelayTimeMs, pTimer->u32PrevMills);
//        SendMsg_RPI((uint8_t*)debugBuffer, strlen(debugBuffer));
//    }
//}




uint8_t Timer_IsTimeout(stcTimer *pTimer , TimerMode m_TimerMode)
{
	uint8_t bStatus = false;
//	char debugBuffer[64];
//	snprintf(debugBuffer, sizeof(debugBuffer), "Timer is running? : %lu cm\n", Timer_IsRunning(pTimer));
//	SendMsg_RPI((uint8_t*)debugBuffer, strlen(debugBuffer));
	if(true == Timer_IsRunning(pTimer))
	{
		pTimer->u32CurrMills = HAL_GetTick();
		int32_t elapsedTime = (int32_t)(pTimer->u32CurrMills - pTimer->u32PrevMills);
		if(abs(elapsedTime) >= pTimer->u32TargetMills)
		{
			bStatus = true;
			if(enTimerAutoRestart == m_TimerMode)
			{
				uint32_t u32TargetMillis = pTimer->u32TargetMills;
				StartTimer(pTimer , u32TargetMillis);
			}
			else/*Normal mode of timer without auto restart*/
			{
				StopTimer(pTimer);
			}
		}
	}
	return bStatus;
}

void StopTimer(stcTimer *pTimer)
{
	InitilizeTimer(pTimer);
}

uint8_t Timer_IsRunning(stcTimer *pTimer)
{
	return pTimer->bEnable;
}
