#ifndef _ENCODER_H_
#define _ENCODER_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "main.h"
#include "disc_calc.h"
#include "cmsis_os2.h"

/* Exported define -----------------------------------------------------------*/
#define	ENCODER_READING_FREQ				(150.0)										/* Hz */
#define ENCODER_READING_PERIOD			(1/ENCODER_READING_FREQ) 		/* Sec. */

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	osTimerId_t 			TIM_PeriodicReading;

}Encoder_OS;

typedef struct
{
	TIM_HandleTypeDef * timHandle;
	uint32_t timChannels;
	
}Encoder_HW;

typedef struct
{
	Encoder_HW hw;
	Encoder_OS os;
	
	uint16_t cpr;
	int16_t  count;
	
	double gain;
	double angOffset;
	double period;
	
	/* Position */
	double currAng;
	double prevAng;
	
	/* Raw Velocity */
	double rawPrevAngVel;
	double rawCurrAngVel;
	
	/* Filtered Velocity */
	double currAngVel;
	double prevAngVel;
	
	/* Lowpass Filter Constants */
	LowpassFilter_Handle velFilter;

}Encoder_Handle;


/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void ENC_Init(Encoder_Handle* encoder);
void ENC_GetAngularData(Encoder_Handle* encoder);

#endif /* _ENCODER_H_ */
