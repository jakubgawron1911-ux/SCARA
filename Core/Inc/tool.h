/*
 * tool.h
 *
 *  Created on: Nov 18, 2025
 *      Author: jakub
 */

#ifndef INC_TOOL_H_
#define INC_TOOL_H_

#include "scara.h"

#define OPEN 1.0
#define CLOSE -1.0

typedef struct {
	uint8_t need_curr;
	float curr;
	I2C_HandleTypeDef *phi2c;
	//slave adres, rejestr
	uint8_t dev_addr, mem_addr, buff[2];
	float R_shunt, sign;
} INA226;
typedef struct {
	//kierunek obrotow
	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;
	//rejest wypelnienia pwm
	volatile uint32_t *ppulse;
	float *psign;
} DC_MOT;
typedef struct {
	float actual, target;
	INA226 ina;
	DC_MOT mot;
	CALMAN cal;
	REG reg;
} TOOL;

void Control2PWM(DC_MOT *, float);
void ToolControlLoop(TOOL *);

#endif /* INC_TOOL_H_ */
