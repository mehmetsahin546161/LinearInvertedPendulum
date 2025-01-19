/* Includes ------------------------------------------------------------------*/
#include "pole_placement.h"
#include "defines.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "disc_calc.h"
#include "app_main.h"
#include "tim.h"
#include "usart.h"

/* Private define ------------------------------------------------------------*/
#define POLE_PLACEMENT_THREAD_STACK_SIZE	512

#define EVT_FLAG_POLE_PLACEMENT_PERIOD_ELAPSED		(1<<0)
#define EVT_FLAG_POLE_PLACEMENT_FAULT							(1<<1)

#define W_TO_CCR(w) (0.087*w + 100)

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
osThreadAttr_t polePlacementThreadAttr = 
{
	.name = "Pole_Placement_Thread",
	.priority = osPriorityAboveNormal2,
};

/* Exported variables --------------------------------------------------------*/
double u;
double sumErr = 0;
double errDiff = 0;
//double prevSumErr = 0;
double prevErr = 0;

double integral_output_max = 0.5;
double integral_output_min = -0.5;

double ref = 0;
double err;
double out;
double prop;
double integ;
double deriv;
char UART_TX_Buff[250];
//float k1 = -9.647, k2 = -26.027, k3 = -78.212, k4 = -15.980;
float k1 = -30.61, k2 = -36.61, k3 = -95.61, k4 = -17.61;

/* Private function prototypes -----------------------------------------------*/
static void PolePlc_ThreadFunc(void* arg);
static void PolePlc_PeriodicTimerFunc(void* arg);
static void PolePlc_CalculateControlSignal(PolePlacement_Handle * controller);
/* Exported functions --------------------------------------------------------*/
/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void PolePlc_Init(PolePlacement_Handle * controller)
{
	/* Create OS resources */
	controller->osResource.EVT_DiscController = osEventFlagsNew(NULL);
	controller->osResource.TIM_DiscController = osTimerNew(PolePlc_PeriodicTimerFunc, osTimerPeriodic, controller, NULL);
	controller->osResource.TID_DiscController = osThreadNew(PolePlc_ThreadFunc, controller, &polePlacementThreadAttr);
	
	PolePlc_EnableController(controller);
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void PolePlc_EnableController(PolePlacement_Handle * controller)
{
	controller->enabled = true;
	osTimerStart(controller->osResource.TIM_DiscController, (uint32_t)SEC_TO_MS(controller->samplingTime));
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void PolePlc_DisableController(PolePlacement_Handle * controller)
{
	osTimerStop(controller->osResource.TIM_DiscController);
	controller->enabled = false;
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
bool PolePlc_IsControllerEnabled(PolePlacement_Handle * controller)
{
	return controller->enabled;
}

/* Private functions ---------------------------------------------------------*/

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static void PolePlc_ThreadFunc(void* arg)
{
	PolePlacement_Handle * controller = (PolePlacement_Handle *)arg;
	uint32_t eventFlags = 0;
	
	float mp = 0.08;
	float mc = 0.3;
	float g = 9.81;
	float Lp = 0.18;
	float Jp = 0.0020;
	float Jeq = Jp + (mp*Lp*Lp);
	float f_theta;
	float b_theta;
	float alpha;
	float alpha_dot;
	float E;
	float E0 = 0;
	float sigma;
	float cond;
	float mu;
	
	while(true)
	{
		eventFlags = osEventFlagsWait(controller->osResource.EVT_DiscController, EVT_FLAG_POLE_PLACEMENT_PERIOD_ELAPSED|EVT_FLAG_POLE_PLACEMENT_FAULT, osFlagsWaitAny, osWaitForever);
		
		if(PolePlc_IsControllerEnabled(controller) == true)
		{
			if(eventFlags & EVT_FLAG_POLE_PLACEMENT_PERIOD_ELAPSED)
			{
				if(fabs(MotorEncoder.currAng) <= 0.3)
				{
					if(fabs(RADIAN_TO_DEGREE(PendulumEncoder.currAng)) <= 15.0)
					{
						/* Stabilization Controller with Full State Feedback */
						u = (-k1*MotorEncoder.currAng) + (-k2*MotorEncoder.currAngVel) + (-k3*PendulumEncoder.currAng) + (-k4*PendulumEncoder.currAngVel);
						
						if(u >= 0)
						{
							MTR_SetDirection(&DC_Motor, MOTOR_TURN_RIGHT);
						}
						else
						{
							u *= -1;
							MTR_SetDirection(&DC_Motor, MOTOR_TURN_LEFT);
						}
						
						MTR_SetSpeed(&DC_Motor, u/12.0);
					}
					else
					{
						/* Swing-Up Controller */
						alpha = PendulumEncoder.currAng;
						alpha_dot = PendulumEncoder.currAngVel;
						
						f_theta = ((-mp*mp*g*Lp*sin(alpha)*cos(alpha)) + (alpha_dot*alpha_dot*mp*Lp*Jeq*sin(alpha)))/((Jeq*(mp+mc)) - (mp*mp*Lp*Lp*cos(alpha)));
						b_theta = Jeq/((Jeq*(mp+mc)) - (mp*mp*Lp*Lp*cos(alpha)));
						E = 0.5*(Jeq*alpha_dot*alpha_dot) + mp*g*Lp*(cos(alpha) - 1);
						sigma = mp*Lp*alpha_dot*cos(alpha)*(E0 - E);
						cond = -f_theta/b_theta; 
						
						if(sigma*b_theta >= 0)
						{
							u = cond - 3;
						}
						else
						{
							u = cond + 3;
						}
						
						if(u >= 0)
						{
							MTR_SetDirection(&DC_Motor, MOTOR_TURN_RIGHT);
						}
						else
						{
							u *= -1;
							MTR_SetDirection(&DC_Motor, MOTOR_TURN_LEFT);
						}
						
						MTR_SetSpeed(&DC_Motor, u/12.0);
					}
				}
				else
				{
					MTR_SetSpeed(&DC_Motor, 0);
				}
				
				
					
				sprintf(UART_TX_Buff, "%.2f %.2f\r\n", MotorEncoder.currAng, PendulumEncoder.currAng);
				UART_AsyncTransmit(&huart6, (uint8_t *)UART_TX_Buff, strlen(UART_TX_Buff));
			}
		}
	}	
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static void PolePlc_PeriodicTimerFunc(void* arg)
{
	PolePlacement_Handle * controller = (PolePlacement_Handle *)arg;
	
	osEventFlagsSet(controller->osResource.EVT_DiscController, EVT_FLAG_POLE_PLACEMENT_PERIOD_ELAPSED);
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note					Integral and reference input signals will be added insALLAH <^-^>		
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static void PolePlc_CalculateControlSignal(PolePlacement_Handle * controller)
{
	/* u[n] = -K*StateVariable -Ki*SumOfError */
	double sumState = 0;
	double sumIntegralErr = 0;
	double err;
	
	for(uint8_t i=0; i<controller->inputCnt; i++)
	{
		/* -K*StateVariable */
		for(uint8_t j=0; j<controller->stateCnt; j++)
		{
			sumState += -1*(controller->K[i][j])*(*(controller->stateVector[j]));
		}
		
		/* -Ki*SumOfError */
		for(uint8_t k=0; k<controller->outputCnt; k++)
		{
			err = controller->ref[k] - *(controller->outputVector[k]);
			sumIntegralErr += -1*(controller->Ki[i][k])*GetDiscreteIntegral(err, &(controller->sumError[i][k]), controller->samplingTime);
		}
		
		controller->inputVector[i] = sumState + sumIntegralErr;
		
		sumState = 0;
		sumIntegralErr = 0;
	}
}
