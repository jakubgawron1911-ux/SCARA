/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scara.h"
#include "tool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
JOINT U1, U2, U3;
TOOL T;

#define BUFF_SIZE 40
uint8_t rx_buff[BUFF_SIZE], tx_buff[BUFF_SIZE];

float u, threshold = 0.15, u_last;
volatile uint8_t u1_done, u2_done, u3_done, tool_done;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PresentationProcess(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  	  ToolInit();
	U1Init();
	U2Init();
	HAL_TIM_Base_Start_IT(&htim2);

	U3Init();
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buff, BUFF_SIZE - 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (U1.meas.need_meas) {
		AS5600 *pas = &U1.meas.sen.enc;
		HAL_I2C_Mem_Read(pas->phi2c, pas->dev_addr << 1, pas->mem_addr, 1, pas->buff, 2, 5);
		UpdateAngleMeas(&U1.meas);
	}

	if (U2.meas.need_meas) {
		AS5600 *pas = &U2.meas.sen.enc;
		HAL_I2C_Mem_Read(pas->phi2c, pas->dev_addr << 1, pas->mem_addr, 1, pas->buff, 2, 5);
		UpdateAngleMeas(&U2.meas);
	} else if (U3.meas.need_meas) {
		MEAS *pmeas = &U3.meas;
		VL53L0X_RangingMeasurementData_t RangingData;
		VL53L0X_Error status = VL53L0X_GetRangingMeasurementData(pmeas->sen.tof.Dev, &RangingData);
		VL53L0X_ClearInterruptMask(pmeas->sen.tof.Dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
		if (status == VL53L0X_ERROR_NONE && RangingData.RangeMilliMeter < 160.0 && RangingData.RangeMilliMeter > 30.0) {
			pmeas->measurement = (float)RangingData.RangeMilliMeter;
		}
		pmeas->need_meas = FALSE;
	} else if (T.ina.need_curr) {
		HAL_I2C_Mem_Read(T.ina.phi2c, T.ina.dev_addr << 1, T.ina.mem_addr, 1, T.ina.buff, 2, 5);
		uint16_t raw_voltage = (int16_t)((T.ina.buff[0] << 8) | T.ina.buff[1]);
		T.ina.curr = ((float)raw_voltage * 2.5e-6 * T.ina.sign) / T.ina.R_shunt;
		T.ina.need_curr = FALSE;
	}

	PresentationProcess();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/* USER CODE BEGIN 4 */
void ToolInit(void) {
	{
		INA226 *pina = &T.ina;
		pina->phi2c = &hi2c2;
		pina->dev_addr = 0x40;
		pina->mem_addr = 0x01;
		pina->R_shunt = 0.1;
		pina->sign = -1.0;
	}
	{
		DC_MOT *pmot = &T.mot;
		pmot->dir_port = DRV_PHASE_GPIO_Port;
		pmot->dir_pin = DRV_PHASE_Pin;
		pmot->ppulse = &TIM1->CCR3;
		pmot->psign = &T.ina.sign;
	}
	{	//Kalman
		CALMAN *pcal = &T.cal;
		pcal->A[0][0] = 1.0; pcal->A[0][1] = 0.007869387;
		pcal->A[1][0] = 0.0; pcal->A[1][1] = 0.6065306;
		pcal->B[0] = 0.000004048165;
		pcal->B[1] = 0.0007475917;
		pcal->V[0][0] = 1e-6;	pcal->V[0][1] = 0.0;
		pcal->V[1][0] = 0.0;	pcal->V[1][1] = 1e-4;
		pcal->W = 0.001;
		pcal->Pe[0][0] = 10.0;
		pcal->Pe[1][1] = 1.0;
		pcal->xe[0] = 0.0;
		pcal->xe[1] = 0.0;
		pcal->last_u = 0.0;
	}
	{	//regulator
		REG *preg = &T.reg;
		preg->K1 = 2675.257; preg->K2 = -139.5136;
		preg->Kr = 2675.257;
	}
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	T.ina.need_curr = TRUE;
}
void U1Init(void) {
	U1.id = 1;

	{	//czujnik kata
		AS5600 *pas = &U1.meas.sen.enc;
		pas->phi2c = &hi2c1;
		pas->dev_addr = 0x36;
		pas->mem_addr = 0x0c;
		pas->lr = 0;
		pas->prev_angle = INIT;
	}

	{	//sterownik silnika
		MOTOR *pmot = &U1.mot;
		pmot->ms1_port = U1_MS1_GPIO_Port;
		pmot->ms1_pin = U1_MS1_Pin;
		pmot->ms2_port = U1_MS2_GPIO_Port;
		pmot->ms2_pin  = U1_MS2_Pin;
		pmot->en_port = U1_EN_GPIO_Port;
		pmot->en_pin  = U1_EN_Pin;
		pmot->dir_port = U1_DIR_GPIO_Port;
		pmot->dir_pin  = U1_DIR_Pin;
		pmot->ptim = TIM17;
		pmot->ppulse = &(TIM17->CCR1);
	}

	{	//Kalman
		CALMAN *pcal = &U1.cal;
		pcal->A[0][0] = 1.0; pcal->A[0][1] = 0.007869387;
		pcal->A[1][0] = 0.0; pcal->A[1][1] = 0.6065306;
		pcal->B[0] = 0.000004048165;
		pcal->B[1] = 0.0007475917;
		pcal->V[0][0] = 1e-6;	pcal->V[0][1] = 0.0;
		pcal->V[1][0] = 0.0;	pcal->V[1][1] = 1e-4;
		pcal->W = 0.001;
		pcal->Pe[0][0] = 10.0;
		pcal->Pe[1][1] = 1.0;
		pcal->xe[0] = 0.0;
		pcal->xe[1] = 0.0;
		pcal->last_u = 0.0;
	}

	{	//regulator
		REG *preg = &U1.reg;
		preg->K1 = 2675.257; preg->K2 = -139.5136;
		preg->Kr = 2675.257;
	}

	{	//rampa
		RAMP *pramp = &U1.ramp;
		pramp->target_value = INIT;
		pramp->state = END;
		pramp->k = 0.7;
		pramp->dt = 0.01;
		//eps, actual, t
	}
	//ustawienie 1/8 krokow
	HAL_GPIO_WritePin(U1.mot.ms1_port, U1.mot.ms1_pin, FALSE);
	HAL_GPIO_WritePin(U1.mot.ms2_port, U1.mot.ms2_pin, FALSE);

	HAL_GPIO_WritePin(U1.mot.en_port, U1.mot.en_pin, FALSE);

	Control2Freq(&U1.mot, 0.0);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

	U1.meas.need_meas = TRUE;
}

void U2Init(void) {
	U2.id = 2;

	{	//czujnik kata
		AS5600 *pas = &U2.meas.sen.enc;
		pas->phi2c = &hi2c2;
		pas->dev_addr = 0x36;
		pas->mem_addr = 0x0c;
		pas->lr = 0;
		pas->prev_angle = INIT;
	}

	{	//sterownik silnika
		MOTOR *pmot = &U2.mot;
		pmot->ms1_port = U2_MS1_GPIO_Port;
		pmot->ms1_pin = U2_MS1_Pin;
		pmot->ms2_port = U2_MS2_GPIO_Port;
		pmot->ms2_pin  = U2_MS2_Pin;
		pmot->en_port = U2_EN_GPIO_Port;
		pmot->en_pin  = U2_EN_Pin;
		pmot->dir_port = U2_DIR_GPIO_Port;
		pmot->dir_pin  = U2_DIR_Pin;
		pmot->ptim = TIM16;
		pmot->ppulse = &(TIM16->CCR1);
	}

	{	//Kalman
		CALMAN *pcal = &U2.cal;
		pcal->A[0][0] = 1.0; pcal->A[0][1] = 0.006321205;
		pcal->A[1][0] = 0.0; pcal->A[1][1] = 0.3678794;
		pcal->B[0] = 0.00001434730;
		pcal->B[1] = 0.002465270;
		pcal->V[0][0] = 1e-6;	pcal->V[0][1] = 0.0;
		pcal->V[1][0] = 0.0;	pcal->V[1][1] = 1e-4;
		pcal->W = 0.001;
		pcal->Pe[0][0] = 10.0;
		pcal->Pe[1][1] = 1.0;
		pcal->xe[0] = 0.0;
		pcal->xe[1] = 0.0;
		pcal->last_u = 0.0;
	}

	{	//regulator
		REG *preg = &U2.reg;
		preg->K1 = 811.2701; preg->K2 = -139.4411;
		preg->Kr = 811.2701;
	}

	{	//rampa
		RAMP *pramp = &U2.ramp;
		pramp->target_value = INIT;
		pramp->state = END;
		pramp->k = 0.6;
		pramp->dt = 0.01;
		//eps, actual, t
	}
	//ustawienie 1/8 krokow
	HAL_GPIO_WritePin(U2.mot.ms1_port, U2.mot.ms1_pin, FALSE);
	HAL_GPIO_WritePin(U2.mot.ms2_port, U2.mot.ms2_pin, FALSE);

	HAL_GPIO_WritePin(U2.mot.en_port, U2.mot.en_pin, FALSE);

	Control2Freq(&U2.mot, 0.0);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

	U2.meas.need_meas = TRUE;
}
void U3Init(void) {
	U3.id = 3;
	//ustawienie maksymalnego pradu
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(1.2 / 3.3 * 4095.0));
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);


	{	//czujnik odleglosci
		VL53L0X *ptof = &U3.meas.sen.tof;
		ptof->Dev = &ptof->vl53l0x_c;
		ptof->Dev->I2cHandle = &hi2c2;
		ptof->Dev->I2cDevAddr = 0x52;
		VL53L0X_WaitDeviceBooted(ptof->Dev);
		VL53L0X_DataInit(ptof->Dev);
		VL53L0X_StaticInit(ptof->Dev);
		uint32_t temp;
		VL53L0X_PerformRefCalibration(ptof->Dev, (uint8_t *)&temp, (uint8_t *)&temp);
		VL53L0X_PerformRefSpadManagement(ptof->Dev, &temp, (uint8_t *)&temp);
		VL53L0X_SetDeviceMode(ptof->Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
		VL53L0X_StartMeasurement(ptof->Dev);
	}

	{	//sterownik silnika
		MOTOR *pmot = &U3.mot;
		pmot->ms1_port = U3_MS1_GPIO_Port;
		pmot->ms1_pin = U3_MS1_Pin;
		pmot->ms2_port = U3_MS2_GPIO_Port;
		pmot->ms2_pin  = U3_MS2_Pin;
		pmot->en_port = U3_EN_GPIO_Port;
		pmot->en_pin  = U3_EN_Pin;
		pmot->dir_port = U3_DIR_GPIO_Port;
		pmot->dir_pin  = U3_DIR_Pin;
		pmot->ptim = TIM3;
		pmot->ppulse = &(TIM3->CCR3);
	}

	{	//Kalman
		CALMAN *pcal = &U3.cal;
		pcal->A[0][0] = 1.0; 	pcal->A[0][1] = 0.03934693;
		pcal->A[1][0] = 0.0; 	pcal->A[1][1] = 0.6065306;
		pcal->B[0] = -0.0002343674;
		pcal->B[1] = -0.008656325;

		float war = 100;
		pcal->V[0][0] = war * 0.0000015625;pcal->V[0][1] = war * 0.0000625;//pcal->V[0][0] = 2.25e-7;pcal->V[0][1] = 4.5e-7;
		pcal->V[1][0] = war * 0.0000625;	pcal->V[1][1] = war * 0.0025;//pcal->V[1][0] = 4.5e-7;	pcal->V[1][1] = 9.0e-6;

		//wariancja 1.1839
		pcal->W = 1.1839;//6.85e-5;

		pcal->Pe[0][0] = 10.0;
		pcal->Pe[1][1] = 1.0;

		pcal->xe[0] = 0.0;
		pcal->xe[1] = 0.0;
		pcal->last_u = 0.0;
	}

	{	//regulator
		REG *preg = &U3.reg;
		preg->K1 = -104.3264; preg->K2 = -19.67882;
		preg->Kr = -104.3264;
	}

	{	//rampa
		RAMP *pramp = &U3.ramp;
		pramp->target_value = INIT;
		pramp->state = END;
		pramp->k = 0.06;
		pramp->dt = 0.05;
	}

	//ustawienie 1/8 krokow
	HAL_GPIO_WritePin(U3.mot.ms1_port, U3.mot.ms1_pin, FALSE);
	HAL_GPIO_WritePin(U3.mot.ms2_port, U3.mot.ms2_pin, FALSE);

	HAL_GPIO_WritePin(U3.mot.en_port, U3.mot.en_pin, FALSE);

	Control2Freq(&U3.mot, 0.0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	U1.meas.need_meas = TRUE;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart3) {
		uint8_t length = 12;
		uint8_t start = BUFF_SIZE + Size - length;
		uint8_t temp_buff[length];
		for (uint8_t i = 0; i < length; i++) {
		    temp_buff[i] = rx_buff[(start + i) % BUFF_SIZE];
		}

		uint32_t rec_crc, cal_crc;
		memcpy(&rec_crc, temp_buff + 8, sizeof(uint32_t));
		cal_crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)temp_buff, 8) ^ 0xffffffff;
		if (cal_crc != rec_crc) {//zle crc
			return;
		}
		uint8_t header = temp_buff[0];
		if (header != INIT) {//zly naglowek
			return;
		}
		float target;
		uint8_t id = temp_buff[1];
		switch (id) {//parsing danych
			case 0: {
				memcpy(&target, temp_buff + 4, sizeof(float));
				if (fabs(target - 1.0) < EPS) {
					u = 10.0;
				} else {
					u = -10.0;
				}
				//logika po otrzymaniou wiadomsoci apropos chwytaka
			} break;
			case 1: {
				if (U1.ramp.state != END) return;
				memcpy(&target, temp_buff + 4, sizeof(float));
				U1.ramp.target_value = target * M_PI / 180.0;
				U1.ramp.state = BEGIN;
			} break;
			case 2: {
				if (U2.ramp.state != END) return;
				memcpy(&target, temp_buff + 4, sizeof(float));
				U2.ramp.target_value = target * M_PI / 180.0;
				U2.ramp.state = BEGIN;
			} break;
			case 3: {
				if (U3.ramp.state != END) return;
				memcpy(&target, temp_buff + 4, sizeof(float));
				target = 2.657e-3 * target * target + 0.797 * target + 34.8;
				U3.ramp.target_value = target;
				U3.ramp.state = BEGIN;
			} break;
			default: {
				return;
			}
		}
	}
}
void onRampFinish(uint8_t id) {
	uint8_t size = sprintf((char *) tx_buff, "%d", id);

	switch (id) {
	case 0:
		tool_done = 1;
		break;
	case 1:
		u1_done = 1;
		break;
	case 2:
		u2_done = 1;
		break;
	case 3:
		u3_done = 1;
		break;
	}

	HAL_UART_Transmit_IT(&huart3, tx_buff, size);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {

		ControlLoop(&U1);

		ControlLoop(&U2);

		TOOL *pt = &T;
		if (fabs(T.ina.curr) > threshold) {
			u = 0.0;
			onRampFinish(0);
		} else if (fabs(u) < 100.0 && fabs(u) > 0.001) {
			if (u > 0.0)
			    u += 10.0;
			else
			    u -= 10.0;
		}
		Control2PWM(&pt->mot, u);
		pt->ina.need_curr = TRUE;

		printf("%ld %f %f\r\n", HAL_GetTick(), T.ina.curr, u);
	} else if (htim == &htim7) {
		ControlLoop(&U3);
		//do realnych 1.845e-3 * (float)RangingData.RangeMilliMeter * (float)RangingData.RangeMilliMeter+ 1.25 * (float)RangingData.RangeMilliMeter - 39.58;
	}
}


void PresentationProcess(void) {

    if (HAL_GetTick() < 5000) {
        return;
    }

    static uint8_t state = 0;
    static uint8_t state_entry = 1;
    float target;
    static uint8_t delay = 0;
    static uint32_t tick, sleep = 2000;

    switch (state) {

    // otworzenie chwytaka i ruch ponad klocki
    case 0:
        if (state_entry) {
        	tool_done = 0;
        	u3_done = 0;

            u = -0.01;

            target = 70.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

            state_entry = 0;
        }

        if (tool_done && u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 1;
                state_entry = 1;
        	}
        }
        break;

    // ustawienie sie nad pierwszym klockiem
    case 1:
        if (state_entry) {
        	u1_done = 0;
        	u2_done = 0;

            U1.ramp.target_value = -1.0995;
            U1.ramp.state = BEGIN;

            U2.ramp.target_value = 1.065;
            U2.ramp.state = BEGIN;

            state_entry = 0;
        }

        if (u1_done && u2_done) {
            state = 2;
            state_entry = 1;
        }
        break;

	//zjechanie po pierwszy klocek
	case 2:
        if (state_entry) {
        	u3_done = 0;

        	target = 100.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

            state_entry = 0;
        }

        if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 3;
                state_entry = 1;
        	}
        }
        break;

	//zacisniecie pierwszego klocka
	case 3:
        if (state_entry) {
        	tool_done = 0;

            u = 0.01;

            state_entry = 0;
        }

        if (tool_done) {
            state = 4;
            state_entry = 1;
        }
        break;

	//jazda do gory
	case 4:
		if (state_entry) {
			u3_done = 0;

			target = 90.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 5;
                state_entry = 1;
        	}
		}
		break;

	//jazda na miejsce wierzy
	case 5:
		if (state_entry) {
			u1_done = 0;
			u2_done = 0;

			U1.ramp.target_value = 0.541;
			U1.ramp.state = BEGIN;

			U2.ramp.target_value = -2.199;
			U2.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u1_done && u2_done) {
			state = 6;
			state_entry = 1;
		}
		break;

	//jazda w dol
	case 6:
		if (state_entry) {
			u3_done = 0;

			target = 100.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 7;
                state_entry = 1;
        	}
		}
		break;

	//otworzenie chwytaka
	case 7:
		if (state_entry) {
        	tool_done = 0;
			u = -0.01;

			state_entry = 0;
		}

		if (tool_done) {
			state = 8;
			state_entry = 1;
		}
		break;

	//jazda w gore
	case 8:
		if (state_entry) {
			u3_done = 0;

			target = 70.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 9;
                state_entry = 1;
        	}
		}
		break;

	//jazda nad drugi klocek
	case 9:
		if (state_entry) {
			u1_done = 0;
			u2_done = 0;

			U1.ramp.target_value = -1.5708;
			U1.ramp.state = BEGIN;

			U2.ramp.target_value = 1.5185;
			U2.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u1_done && u2_done) {
			state = 10;
			state_entry = 1;
		}
		break;

	//jazda w gore
	case 10:
		if (state_entry) {
			u3_done = 0;

			target = 100.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 11;
                state_entry = 1;
        	}
		}
		break;

	//otworzenie chwytaka
	case 11:
		if (state_entry) {
			tool_done = 0;
			u = 0.01;

			state_entry = 0;
		}

		if (tool_done) {
			state = 12;
			state_entry = 1;
		}
		break;

	//jazda w gore
	case 12:
		if (state_entry) {
			u3_done = 0;

			target = 60.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 13;
                state_entry = 1;
        	}
		}
		break;

	//jazda na miejsce wierzy
	case 13:
		if (state_entry) {
			u1_done = 0;
			u2_done = 0;

			U1.ramp.target_value = 0.541;
			U1.ramp.state = BEGIN;

			U2.ramp.target_value = -2.199;
			U2.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u1_done && u2_done) {
			state = 14;
			state_entry = 1;
		}
		break;

	//jazda w gore
	case 14:
		if (state_entry) {
			u3_done = 0;

			target = 70.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 15;
                state_entry = 1;
        	}
		}
		break;

	//otworzenie chwytaka
	case 15:
		if (state_entry) {
			tool_done = 0;
			u = -0.01;

			state_entry = 0;
		}

		if (tool_done) {
			state = 16;
			state_entry = 1;
		}
		break;

	//jazda w gore
	case 16:
		if (state_entry) {
			u3_done = 0;

			target = 40.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 17;
                state_entry = 1;
        	}
		}
		break;

	//jazda na miejsce wierzy
	case 17:
		if (state_entry) {
			u1_done = 0;
			u2_done = 0;

			U1.ramp.target_value = -1.972;
			U1.ramp.state = BEGIN;

			U2.ramp.target_value = 1.5185;
			U2.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u1_done && u2_done) {
			state = 18;
			state_entry = 1;
		}
		break;

	//jazda w gore
	case 18:
		if (state_entry) {
			u3_done = 0;

			target = 100.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 19;
                state_entry = 1;
        	}
		}
		break;

	//otworzenie chwytaka
	case 19:
		if (state_entry) {
			tool_done = 0;
			u = 0.01;

			state_entry = 0;
		}

		if (tool_done) {
			state = 20;
			state_entry = 1;
		}
		break;

	//jazda w gore
	case 20:
		if (state_entry) {
			u3_done = 0;

			target = 30.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 21;
                state_entry = 1;
        	}
		}
		break;

	//jazda na miejsce wierzy
	case 21:
		if (state_entry) {
			u1_done = 0;
			u2_done = 0;

			U1.ramp.target_value = 0.541;
			U1.ramp.state = BEGIN;

			U2.ramp.target_value = -2.199;
			U2.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u1_done && u2_done) {
			state = 22;
			state_entry = 1;
		}
		break;

	//jazda w gore
	case 22:
		if (state_entry) {
			u3_done = 0;

			target = 40.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 23;
                state_entry = 1;
        	}
		}
		break;

	case 23:
		if (state_entry) {
			tool_done = 0;
			u = -0.01;

			state_entry = 0;
		}

		if (tool_done) {
			state = 24;
			state_entry = 1;
		}
		break;

	//jazda w gore
	case 24:
		if (state_entry) {
			u3_done = 0;

			target = 10.0;
			target = 2.657e-3 * target * target + 0.797 * target + 34.8;
			U3.ramp.target_value = target;
			U3.ramp.state = BEGIN;

			state_entry = 0;
		}

		if (u3_done) {
        	if (delay == 0) {
        		delay = 1;
        		tick = HAL_GetTick();
        	} else if (HAL_GetTick() - tick > sleep) {
        		delay = 0;;
                state = 25;
                state_entry = 1;
        	}
		}
		break;

		//jazda na miejsce wierzy
		case 25:
			if (state_entry) {
				u1_done = 0;
				u2_done = 0;

				U1.ramp.target_value = -1.8;
				U1.ramp.state = BEGIN;

				U2.ramp.target_value = 1.57;
				U2.ramp.state = BEGIN;

				state_entry = 0;
			}

			if (u1_done && u2_done) {
				state = 26;
				state_entry = 1;
			}
			break;
	default:
		return;
		break;
    }
}
/************************** DEBUG **************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == SAFETY_Pin) {
		static uint32_t last_tick = 101;
	    if ((HAL_GetTick() - last_tick) > 100) {
	    	HAL_GPIO_TogglePin(U1_EN_GPIO_Port, U1_EN_Pin);
	    	HAL_GPIO_TogglePin(U2_EN_GPIO_Port, U2_EN_Pin);
	    	HAL_GPIO_TogglePin(U3_EN_GPIO_Port, U3_EN_Pin);
	    	u = 0.0;
			Control2PWM(&T.mot, 0);

	        last_tick = HAL_GetTick();
	    }
	}
}

int _write(int file, char *ptr, int len) {
	return HAL_UART_Transmit_IT(&huart2, (uint8_t *)ptr, len);
}
/************************** DEBUG **************************/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(U1_EN_GPIO_Port, U1_EN_Pin, TRUE);
	HAL_GPIO_WritePin(U2_EN_GPIO_Port, U2_EN_Pin, TRUE);
	HAL_GPIO_WritePin(U3_EN_GPIO_Port, U3_EN_Pin, TRUE);
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
