/* Includes ------------------------------------------------------------------*/
#include "encoder.h"
#include "cmsis_os2.h"
#include "defines.h"
#include "disc_calc.h"
#include "app_main.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"


/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
const osTimerAttr_t periodicTimerAttr = 
{
	.name = "Encoder_Periodic_Timer"
};

/* Private function prototypes -----------------------------------------------*/
static void ENC_PeriodicTimerFunc(void* arg);

/* Private functions ---------------------------------------------------------*/


/* Exported functions --------------------------------------------------------*/

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ENC_Init(Encoder_Handle* encoder)
{
	HAL_TIM_Encoder_Start(encoder->hw.timHandle, encoder->hw.timChannels);
	
	encoder->os.TIM_PeriodicReading = osTimerNew(ENC_PeriodicTimerFunc, osTimerPeriodic, encoder, &periodicTimerAttr);
	osTimerStart(encoder->os.TIM_PeriodicReading, SEC_TO_MS(encoder->period));
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void ENC_GetAngularData(Encoder_Handle * encoder)
{
	uint32_t tempCnt = __HAL_TIM_GET_COUNTER(encoder->hw.timHandle);
	encoder->count = (int16_t)(tempCnt);
	
	/* Angular Position */
	encoder->currAng = ((float)(encoder->count) / (encoder->cpr))*2*PI;
	encoder->currAng *= encoder->gain;
	encoder->currAng -= encoder->angOffset;
	
	/* Raw Angular Velocity */
	encoder->rawCurrAngVel = GetDiscreteDerivative(encoder->currAng, &(encoder->prevAng), encoder->period);
	
	/* Filtered Angular Velocity */
	encoder->currAngVel = ApplyLowpassFilter(&(encoder->velFilter), encoder->rawCurrAngVel);
}

/* Private functions ---------------------------------------------------------*/

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static void ENC_PeriodicTimerFunc(void* arg)
{
	Encoder_Handle * encoder = (Encoder_Handle*)(arg);
	ENC_GetAngularData(encoder);
}