#ifndef _APP_H_
#define _APP_H_

/* Includes ------------------------------------------------------------------*/
#include "encoder.h"
#include "dc_motor.h"
#include "pole_placement.h"
#include "gpio.h"
#include "tim.h"

/* Exported define -----------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern Encoder_Handle MotorEncoder;
extern Encoder_Handle PendulumEncoder;
extern DC_Motor_Handle DC_Motor;
extern PolePlacement_Handle FSFBController;

/* Exported functions --------------------------------------------------------*/
void APP_Main(void* arg); 


#endif /* _APP_H_ */
