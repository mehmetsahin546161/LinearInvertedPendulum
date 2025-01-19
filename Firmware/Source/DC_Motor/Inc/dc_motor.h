#ifndef _MOTOR_H_
#define _MOTOR_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "main.h"

/* Exported define -----------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

typedef enum
{
	MOTOR_TURN_LEFT,
	MOTOR_TURN_RIGHT

}MotorDirection;

typedef struct
{
	GPIO_TypeDef * DirGPIO;
	uint16_t DirPinA;
	uint16_t DirPinB;
	TIM_HandleTypeDef * Timer;
	uint32_t TimerChannel;
	uint32_t TimerPeriod;		/* In STM devices, it is ARR value */
	
}DC_Motor_HWInit;

typedef struct
{
	DC_Motor_HWInit Init;
	
}DC_Motor_Handle;


/* Exported functions --------------------------------------------------------*/
void MTR_Init(DC_Motor_Handle * motor);
void MTR_SetDirection(DC_Motor_Handle * motor, MotorDirection direction);
void MTR_SetSpeed(DC_Motor_Handle * motor, float percSpeed);

#endif /* _MOTOR_H_ */
