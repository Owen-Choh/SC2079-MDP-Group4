/*
 *Author : Aldrin.Rebellow
 */

#include "Drv_HCSR04.h"

const HCSR04_CfgType HCSR04_CfgParam[HCSR04_UNITS] =
{
    // HC-SR04 Sensor Unit 1 Configurations
    {
	GPIOB,
	GPIO_PIN_4,
	TIM4,
	TIM_CHANNEL_2,
	16
    }
};
