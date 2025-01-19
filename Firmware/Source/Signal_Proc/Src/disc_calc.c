#include "disc_calc.h"


/* Exported functions --------------------------------------------------------*/

/**------------------------------------------------------------------------------
  * @brief  			Calcuulates discrete derivative based on Euler backward method.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
double GetDiscreteDerivative(double currVal, double * prevVal, double samplingTime)
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
	
	double out = (currVal - *prevVal)/samplingTime;
	*prevVal = currVal;
	return out;
}

/**------------------------------------------------------------------------------
  * @brief  			Calculates discrete integral based on Euler backward method.
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
double GetDiscreteIntegral(double currVal, double * prevSum, double samplingTime)
{
	//              --------------
	//							|					   |
	//	u[n] ---->  |  Integral  |  ----> y[n]
	//              |            |
	//							--------------
	//
	//  y[n] = y[n-1] + T*u[n]
	
	double out = (*prevSum) + ( samplingTime * (currVal) );
	*prevSum = out;
	return out;
}

/**------------------------------------------------------------------------------
  * @brief  			
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *------------------------------------------------------------------------------*/
double ApplyLowpassFilter(LowpassFilter_Handle * filter, double currRawVal)
{
	double filtVal = (filter->alpha*currRawVal) + ( (1-(filter->alpha))*filter->prevFiltVal );
	filter->prevFiltVal = filtVal;
	
	return filtVal;
}