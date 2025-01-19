#ifndef _POLE_PLACEMENT_H_
#define _POLE_PLACEMENT_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "cmsis_os2.h"
#include "stdbool.h"

/* Exported define -----------------------------------------------------------*/
#define MAX_STATE_COUNT			15
#define MAX_INPUT_COUNT			10
#define MAX_OUTPUT_COUNT		10
	
#define DISC_CTRL_FREQ			(150.0)									/* Hz */
#define DISC_CTRL_PERIOD		(1/DISC_CTRL_FREQ)			/* Sec */


/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	osThreadId_t 			TID_DiscController;
	osTimerId_t 			TIM_DiscController;
	osEventFlagsId_t 	EVT_DiscController;

}PolePlacement_OS_Resource;

typedef struct
{
	bool 	enabled;
	float samplingTime;	//Second
	
	/*

							-------------------------------
							|                             |
							|   x[n+1] = A*x[n] + B*u[n]  |
	u[n] ------>|                             |------> y[n]
							|   y[n] = C*x[n] + D*u[n]    |
							|                            	|
							-------------------------------
	*/
	
	uint8_t inputCnt;
	uint8_t outputCnt;
	uint8_t stateCnt;
	
	double inputVector[MAX_INPUT_COUNT];			// u[n]
	double * stateVector[MAX_STATE_COUNT];		// x[n]
	double * outputVector[MAX_OUTPUT_COUNT];	// y[n]
	double ref[MAX_OUTPUT_COUNT];
	double sumError[MAX_INPUT_COUNT][MAX_OUTPUT_COUNT];
	
	/* State feedback gains */
	double K[MAX_INPUT_COUNT][MAX_STATE_COUNT];
	
	/* Integrator gains */
	double Ki[MAX_INPUT_COUNT][MAX_OUTPUT_COUNT];
	
	/* RTOS related resources */
	PolePlacement_OS_Resource osResource;
	
}PolePlacement_Handle;

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void PolePlc_Init(PolePlacement_Handle * controller);
void PolePlc_EnableController(PolePlacement_Handle * controller);
void PolePlc_DisableController(PolePlacement_Handle * controller);
bool PolePlc_IsControllerEnabled(PolePlacement_Handle * controller);

#endif /* _POLE_PLACEMENT_H_ */
