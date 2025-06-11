/*
 * servo.h
 *
 *  Created on: Jun 11, 2025
 *      Author: krushna
 */

#ifndef SERVO_H_
#define SERVO_H_

#include<stm32f4xx.h>

#define PCLK  16000000
#define psc   16
#define SERVO_PIN   0

void servo_init();
void servo_angle(uint16_t CCR);

#endif /* SERVO_H_ */
