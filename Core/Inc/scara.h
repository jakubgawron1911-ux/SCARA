/*
 * header_file.h
 *
 *  Created on: Aug 31, 2025
 *      Author: jakub
 */

#ifndef INC_SCARA_H_
#define INC_SCARA_H_

#include "stm32f0xx_hal.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "vl53l0x_api.h"

#define FHSE 48000000.0
#define EPS 1e-10
#define TRUE 1
#define FALSE 0

#define CCW 0
#define CW 1
#define MAXARR 65535.0
#define MS 0.125

#define TIMEOUT 10
#define FULLROT (2.0 * M_PI)
#define RES12 4096.0
#define OFFSET M_PI
#define INIT 170.0

#define THETAEPS 1e-3

typedef enum {
	BEGIN = 0,
	RUNNING = 1,
	END = 2
} ramp_state;

typedef struct{
	I2C_HandleTypeDef *phi2c;
	//slave adres, rejestr
	uint8_t dev_addr, mem_addr, buff[2];
	//zmienne do kontroli pelnych obrotow
	int8_t lr;
	float prev_angle;
} AS5600;

typedef struct{
	VL53L0X_RangingMeasurementData_t RangingData;
	VL53L0X_Dev_t  vl53l0x_c;
	VL53L0X_DEV    Dev;//Dev = &vl53l0x_c;
} VL53L0X;


typedef union {
	AS5600 enc;
	VL53L0X tof;
} SENSOR;

typedef struct {
	SENSOR sen;
	float measurement;
	uint8_t need_meas;
} MEAS;

typedef struct {
	//mikrokroki
	GPIO_TypeDef *ms1_port;
	uint16_t ms1_pin;
	GPIO_TypeDef *ms2_port;
	uint16_t ms2_pin;
	//blokowanie wyjsc
	GPIO_TypeDef *en_port;
	uint16_t en_pin;
	//kontorla obrotu (index)
	uint16_t index_pin;
	//kierunek obrotow
	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;
	//rejestry timera do czestotliwosci step
	TIM_TypeDef *ptim;
	volatile uint32_t *ppulse;
} MOTOR;
typedef struct {
	//rownania stanu
	float A[2][2], B[2];
	//macierze kowariancji szumow
	float V[2][2], W;
	//stany estymowane i macierz estymacji
	float xe[2], Pe[2][2];
	//poprzednie sterowanie
	float last_u;
} CALMAN;
typedef struct {
	//parametry regulatora ze sprzezeniem od stanu
	float K1, K2, Kr;
} REG;
typedef struct {
	ramp_state state;
	TIM_HandleTypeDef *phtim;
	float k, eps, actual_value, target_value;
	float t, dt;

	float ramp;
} RAMP;
typedef struct {
	uint8_t id;

	MEAS meas;
	MOTOR mot;

	RAMP ramp;
	CALMAN cal;
	REG reg;
} JOINT;

void U1Init(void);
void U2Init(void);
void U3Init(void);
void ToolInit(void);

void UpdateAngleMeas(MEAS *);
void Control2Freq(MOTOR *, float);

void onRampFinish(uint8_t);
float ComputeRamp(JOINT *);
float sCurveFunc(float, float, float);

float StateFeedbackController(REG *, float, float, float);
float *CalmanFilter(CALMAN *, float);

void ControlLoop(JOINT *);

void mul2x2(float [2][2], float [2][2], float [2][2]);
void trans2x2(float [2][2], float [2][2]);
void add2x2(float [2][2], float [2][2], float R[2][2]);

#endif /* INC_SCARA_H_ */
