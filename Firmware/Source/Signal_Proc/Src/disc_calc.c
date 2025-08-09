#include "disc_calc.h"


/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief  			Calcuulates discrete derivative based on Euler backward method.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
float GetDiscreteDerivative(float currVal, float * prevVal, float samplingTime)
{
	//              ----------------
	//							|						   |
	//	u[n] ---->  |  Derivative  |  ----> y[n]
	//              |              |
	//							----------------
	// 
	//   				 u[n] - u[n-1]
	//  y[n] =	---------------
	//					       T         
	
	float out = (currVal - *prevVal)/samplingTime;
	*prevVal = currVal;
	return out;
}

/**------------------------------------------------------------------------------
  * @brief  			Calculates discrete integral based on Euler backward method.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
float GetDiscreteIntegral(float currVal, float * prevSum, float samplingTime)
{
	//              --------------
	//							|					   |
	//	u[n] ---->  |  Integral  |  ----> y[n]
	//              |            |
	//							--------------
	//
	//  y[n] = y[n-1] + T*u[n]
	
	float out = (*prevSum) + ( samplingTime * (currVal) );
	*prevSum = out;
	return out;
}

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
float ApplyLowpassFilter(LowpassFilter_Handle * filter, float currRawVal)
{
	float filtVal = (filter->alpha*currRawVal) + ( (1-(filter->alpha))*filter->prevFiltVal );
	filter->prevFiltVal = filtVal;
	
	return filtVal;
}