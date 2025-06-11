/*
 * servo.c
 *
 *  Created on: Jun 11, 2025
 *      Author: krushna
 */

//---used timer 2 (General purpose timer)
//---Alternate function 1 ( used pin a0)

#include"servo.h"

void servo_init()
{
	//  pa-> 0
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA ->MODER |= BV(SERVO_PIN +1);
	GPIOA ->MODER &= ~BV(SERVO_PIN);
	GPIOA ->PUPDR &= ~(BV(SERVO_PIN +1) | BV(SERVO_PIN));
	GPIOA ->AFR[0] |= (1 << (0 * 4));

	//timer 2

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = psc - 1;
}


void  servo_angle(uint16_t CCR)
{

	TIM2->ARR = 20000-1;
	TIM2->CCR1 = CCR;

	TIM2 ->CCMR1 &= ~(TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC1S_1);
	TIM2 ->CCER &= ~TIM_CCER_CC1P;

	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->CR1 &= ~(TIM_CR1_CMS_0 | TIM_CR1_CMS_1);
	TIM2->CCER |= TIM_CCER_CC1E;
	TIM2->BDTR |= TIM_BDTR_MOE;
	TIM2->CR1 |= TIM_CR1_CEN;


}

