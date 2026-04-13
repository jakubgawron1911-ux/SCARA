/*
 * source_file.c
 *
 *  Created on: Aug 31, 2025
 *      Author: jakub
 */


#include <scara.h>

void UpdateAngleMeas(MEAS *pmeas) {

	//status komunikacji, odczyt kata (1, 2), obecnosc magnesu (0)
	//uint8_t comm_state, buff[3];
	//comm_state = HAL_I2C_Mem_Read(psen->phi2c, psen->dev_addr << 1, psen->mem_addr, 1, buff, 3, TIMEOUT);
	//if (comm_state != HAL_OK  || !(buff[0] & 0x20)) {
	//	*penc_err = TRUE;
	//	return 0.0;
	//}
	//*penc_err = FALSE;
	//konwersja na kat
	AS5600 *psen = &pmeas->sen.enc;
	float angle;
	uint16_t raw_angle = (psen->buff[0] << 8) | psen->buff[1];
	angle = raw_angle * FULLROT / RES12 - OFFSET;

	if (fabs(psen->prev_angle - INIT) > EPS) {
		if (angle + 0.9 * FULLROT < psen->prev_angle) {
			psen->lr += 1;
		} else if (angle - 0.9 * FULLROT > psen->prev_angle) {
			psen->lr -= 1;
		}
	}
	psen->prev_angle = angle;

	pmeas->measurement = angle + psen->lr * FULLROT;
	pmeas->need_meas = FALSE;
}

void Control2Freq(MOTOR *pmot, float u) {
	//uwzglednienie mikrokrokow w sterowaniu
	float freq = 1.0 / (2.0 * MS) * u;
	//jesli czestotliwosc bardzo bliska zeru ustawienie wypelnienia PWM na 0 i zignorowanie reszty kodu
	if (fabs(freq) < EPS) {
		*(pmot->ppulse) = 0;
		return;
	}
	//ustawienie kierunku obrotow zgodnie ze znakiem sterowania
	HAL_GPIO_WritePin(pmot->dir_port, pmot->dir_pin, (freq > 0.0) ? CW : CCW);
	//obliczenie ARR (PSC = ARR)
	int32_t ARR = (int32_t)round(sqrt(FHSE / fabs(freq)) - 1.0);
	//saturacja potencjalnego ARR
	if (ARR < 0) {
		ARR = 0;
	} else if (ARR > MAXARR) {
		ARR = MAXARR;
	}
	//jesli licznik przeliczyl nowe ARR zresetowanie go
	if (pmot->ptim->CNT > ARR) {
		pmot->ptim->CNT = 0;
	}
	pmot->ptim->PSC = ARR;
	pmot->ptim->ARR = ARR;
	*(pmot->ppulse) = ARR / 2.0;
}
__weak void onRampFinish(uint8_t id) {
	UNUSED(id);
}
float ComputeRamp(JOINT *pjoint) {
	RAMP *pramp = &pjoint->ramp;
	switch (pramp->state) {
		case BEGIN: {
			pramp->actual_value = pjoint->cal.xe[0];
			pramp->t = 0.0;
			pramp->eps = pramp->target_value - pjoint->cal.xe[0];
			pramp->state = RUNNING;
			return pramp->actual_value;
		}
		case RUNNING: {
			//funkcja wyliczajaca rampe przyspieszenia
			float den = fabs(pramp->eps) * pramp->k;
			//obsluga wyliczenia szczytu rampy i dzielenia przez zero
			if (den < pramp->t || fabs(den) < EPS) {
				//weak functio z odeslanie do pico ze mozna nowa watrtosc zadawac
				onRampFinish(pjoint->id);
				pramp->state = END;
				return pramp->target_value;
			}
			float s = pramp->t / den;
			pramp->t += pramp->dt;
			return sCurveFunc(pramp->actual_value, pramp->target_value, s);
		} break;
		default:
			return pramp->target_value;
	}
}
float sCurveFunc(float a, float t, float s) {
	return a + (t - a) * (3.0 * (s * s) - 2.0 * (s * s * s));
}

float StateFeedbackController(REG *preg, float x1, float x2, float setpoint) {
	float u = -preg->K1 * x1;
	u += -preg->K2 * x2;
	u += preg->Kr * setpoint;
	return u;
}

float *CalmanFilter(CALMAN *pcal, float y) {
	//obliczenie stanow przewidywanych
	float xp[2] = {
			pcal->A[0][1] * pcal->xe[1] + pcal->A[0][0] * pcal->xe[0] + pcal->B[0] * pcal->last_u,
			pcal->A[1][1] * pcal->xe[1] + pcal->A[1][0] * pcal->xe[0] + pcal->B[1] * pcal->last_u
	};
	//obliczenie macierzy przewidywan
	float Pp[2][2];
	{
		float A_Pe[2][2], AT[2][2], A_Pe_AT[2][2];
		mul2x2(pcal->A, pcal->Pe, A_Pe);
		trans2x2(pcal->A, AT);
		mul2x2(A_Pe, AT, A_Pe_AT);
		add2x2(A_Pe_AT, pcal->V, Pp);
	}
	//wykonanie pomiaru i obliczenie roznicy miedzy pomiarem a przewidywaniem
	float eps = y - xp[0];
	//obliczenie stanow estymowanych
	float S = Pp[0][0] + pcal->W;
	if (fabs(S) < EPS) {//obsluga S = 0
		S = (S >= 0 ? EPS : -EPS);
	}
	float Kk[2] = {
			Pp[0][0] / S,
			Pp[1][0] / S,
	};
	pcal->xe[0] = xp[0] + Kk[0] * eps;
	pcal->xe[1] = xp[1] + Kk[1] * eps;
	//obliczenie macierzy estymacji
	{
		float ImKk_C[2][2] = {//I-Kk*C
				{1 - Kk[0], 0},
				{	-Kk[1], 1}
		};
		mul2x2(ImKk_C, Pp, pcal->Pe);
	}

	return pcal->xe;
}

void ControlLoop(JOINT *pjoint) {
	//aktualizowanie wartosci zadanej przez rampe
	float target_value = ComputeRamp(pjoint);
	pjoint->ramp.ramp = target_value;
	//Odczyt wyjscia
	float y = pjoint->meas.measurement;

	//Estymacja pozycji i predkosci
	float *pxe = CalmanFilter(&pjoint->cal, y);

	if (fabs(target_value - INIT) > EPS) {
		float u = StateFeedbackController(&pjoint->reg, *pxe, *(pxe + 1), target_value);
		Control2Freq(&pjoint->mot, u);
		pjoint->cal.last_u = u;
	}
	pjoint->meas.need_meas = TRUE;
}
//matematyczne
void mul2x2(float A[2][2], float B[2][2], float R[2][2]) {
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			R[i][j] = 0.0f;
			for (int k = 0; k < 2; k++) {
				R[i][j] += A[i][k] * B[k][j];
			}
		}
	}
}
void trans2x2(float A[2][2], float R[2][2]) {
	R[0][0] = A[0][0];
	R[0][1] = A[1][0];
	R[1][0] = A[0][1];
	R[1][1] = A[1][1];
}
void add2x2(float A[2][2], float B[2][2], float R[2][2]) {
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			R[i][j] = A[i][j] + B[i][j];
		}
	}
}
