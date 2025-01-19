/* Includes ------------------------------------------------------------------*/
#include "dc_motor.h"

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void MTR_Init(DC_Motor_Handle * motor)
{
	HAL_TIM_PWM_Start(motor->Init.Timer, motor->Init.TimerChannel);
	MTR_SetSpeed(motor, 0);
	MTR_SetDirection(motor, MOTOR_TURN_RIGHT);
}

/**------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void MTR_SetDirection(DC_Motor_Handle * motor, MotorDirection direction)
{
	GPIO_PinState pinAState = (direction==MOTOR_TURN_LEFT)?GPIO_PIN_RESET:GPIO_PIN_SET;
	GPIO_PinState pinBState = (direction==MOTOR_TURN_LEFT)?GPIO_PIN_SET:GPIO_PIN_RESET;
	
	HAL_GPIO_WritePin(motor->Init.DirGPIO, motor->Init.DirPinA, pinAState);
	HAL_GPIO_WritePin(motor->Init.DirGPIO, motor->Init.DirPinB, pinBState);
}

/**------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	percSpeed Must be between 0 and 1.
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
void MTR_SetSpeed(DC_Motor_Handle * motor, float percSpeed)
{
	percSpeed = (percSpeed<0)?0:percSpeed;
	percSpeed = (percSpeed>1)?1:percSpeed;
	
	uint32_t cntVal = motor->Init.TimerPeriod * percSpeed;
		
	__HAL_TIM_SET_COMPARE(motor->Init.Timer, motor->Init.TimerChannel, cntVal);	
}