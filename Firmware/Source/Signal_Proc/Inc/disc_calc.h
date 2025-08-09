#ifndef DISC_CALC_H
#define DISC_CALC_H

#include "stdint.h"

/* Exported constants --------------------------------------------------------*/
#define PI  (3.141592)

#define SIGN_BIT_IN_BYTE  		7
#define SIGN_BIT_IN_HALFWORD  15
#define SIGN_BIT_IN_WORD  		31

#define SIGN_BIT_MASK_IN_BYTE  			(1<<SIGN_BIT_IN_BYTE)
#define SIGN_BIT_MASK_IN_HALFWORD  	(1<<SIGN_BIT_IN_HALFWORD)
#define SIGN_BIT_MASK_IN_WORD  			(1<<SIGN_BIT_IN_WORD)

/* Exported macro ------------------------------------------------------------*/
#define RADIAN_TO_DEGREE(X)  		(X*180.0/PI)
#define DEGREE_TO_RADIAN(X)  		(X*PI/180.0)

/* Exported types ------------------------------------------------------------*/

typedef struct
{
	float alpha;
	float prevFiltVal;

}LowpassFilter_Handle;

/* Exported functions --------------------------------------------------------*/
float GetDiscreteDerivative(float currVal, float * prevVal, float samplingTime);
float GetDiscreteIntegral(float currVal, float * prevSum, float samplingTime);
float ApplyLowpassFilter(LowpassFilter_Handle * filter, float currRawVal);


#endif	/* DISC_CALC_H */