/* Includes ------------------------------------------------------------------*/
#include "app_main.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include "disc_calc.h"
#include "usart.h"
#include "string.h"

/* Private define ------------------------------------------------------------*/
#define PENDULUM_ENCODER_CPR		4096
#define MOTOR_ENCODER_CPR				48

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Encoder_Handle MotorEncoder = 
{
	.hw.timHandle = &htim3,
	.hw.timChannels = TIM_CHANNEL_1|TIM_CHANNEL_2,
	.cpr = MOTOR_ENCODER_CPR,
	.gain = 0.0095/19.1, /* teta_out = teta_in/n & x=teta_out*rw => gain = rw/n */
	.angOffset = 0,
	.period = ENCODER_READING_PERIOD,
	.velFilter.alpha = 0.15,//0.4
};

Encoder_Handle PendulumEncoder = 
{
	.hw.timHandle = &htim1,
	.hw.timChannels = TIM_CHANNEL_1|TIM_CHANNEL_2,
	.cpr = PENDULUM_ENCODER_CPR,
	.gain = 1*92.0/90.0,//90 derecede, 2 derece sapiyor.
	.angOffset = -PI,  
	.period = ENCODER_READING_PERIOD,
	.velFilter.alpha = 0.12
};

DC_Motor_Handle DC_Motor = 
{
	.Init.Timer = &htim2,
	.Init.TimerChannel = TIM_CHANNEL_2,
	.Init.TimerPeriod = 1000,
	.Init.DirGPIO = GPIOA,
	.Init.DirPinA = MOTOR_DIR_A_Pin,
	.Init.DirPinB = MOTOR_DIR_B_Pin
};

PolePlacement_Handle PolePlacement =
{
	.samplingTime = DISC_CTRL_PERIOD,
	.inputCnt  = 1,
	.outputCnt = 2,
	.stateCnt  = 4,
	
//	.stateVector[0] = &(motorLeftEncoder.currAngle),
//	.stateVector[1] = &(motorLeftEncoder.currAngularVelocity),
//	.stateVector[2] = &(AHRS.eulerAngles.roll),
//	.stateVector[3] = &(AHRS.bodyRate.p),
//	
//	.outputVector[0] = &(motorLeftEncoder.currAngle),
//	.outputVector[1] = &(AHRS.eulerAngles.roll),
	
	.ref[0] = 0.7,
	.ref[1] = 0,
	
	.K[0][0] = -9.26,
	.K[0][1] = -17.51,
	.K[0][2] = 39.21,
	.K[0][3] = 6.8,
	
	.Ki[0][0] = 1,
	.Ki[0][1] = -1,
};

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void APP_Main(void* arg)
{
	MTR_Init(&DC_Motor);
	ENC_Init(&MotorEncoder);
	ENC_Init(&PendulumEncoder);
	osDelay(1000);
	PolePlc_Init(&PolePlacement);
	
  while(1)
	{
		
  }

}