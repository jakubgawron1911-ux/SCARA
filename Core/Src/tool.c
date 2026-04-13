/*
 * tool.c
 *
 *  Created on: Nov 18, 2025
 *      Author: jakub
 */


#include "tool.h"

void Control2PWM(DC_MOT *pmot, float u) {
	if (u > 0.0) {
		HAL_GPIO_WritePin(pmot->dir_port, pmot->dir_pin, CW);
		*(pmot->psign) = 1.0;
	} else {
		HAL_GPIO_WritePin(pmot->dir_port, pmot->dir_pin, CCW);
		*(pmot->psign) = -1.0;
	}
	u = fabs(u);
	if (u > 99.0) u = 99.0;
	*pmot->ppulse = (volatile uint32_t)u;
}

void ToolControlLoop(TOOL *pt) {
	float target = 0.12;
	float y = pt->ina.curr;
	float *pxe = CalmanFilter(&pt->cal, y);
	float u = StateFeedbackController(&pt->reg, *pxe, *(pxe + 1), target);
	pt->cal.last_u = u;
	Control2PWM(&pt->mot, u);
	pt->ina.need_curr = TRUE;
}
