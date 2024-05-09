/* ============================================================
          INCLUDE
============================================================ */

#ifndef PROJECT_H_
#define PROJECT_H_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "stm32l1xx_hal.h"
#include "stm32l1xx.h"
#include "stm32l1xx_it.h"
#include "stm32l1xx_hal_tim.h"
#include "main.h"
#include "stm32l1xx_hal.h"

#define MAX_PACKET_SIZE 50


typedef struct _EnvacIOStruct {
    uint8_t light_bright;
    uint8_t pir_recv;
    int temperature;
    uint8_t tx_data[MAX_PACKET_SIZE];
}EnvacIOStruct;




#endif /* PROJECT_H_ */