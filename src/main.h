/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H


#ifdef __cplusplus
extern "C" {
#endif
#include "pin_defs.h"
#include <stdint.h>

#define UART_COM Serial3
#define UART_ENC Serial1

#define ENC_SPEED 2500000
#define PC_SPEED 115200

//TODO - move to gpio lib
#if defined(PCB_3200)
#define LED1_ON 
#define LED1_OFF
#define LED2_ON 
#define LED2_OFF
#define LED3_ON 
#define LED3_OFF
#define LED4_ON 
#define LED4_OFF
#elif defined(PCB_5410)
#define LED1_ON  digitalWrite(LED1, LOW);
#define LED1_OFF digitalWrite(LED1, HIGH);
#define LED2_ON  digitalWrite(LED2, LOW);
#define LED2_OFF digitalWrite(LED2, HIGH);
#define LED3_ON  digitalWrite(LED3, LOW);
#define LED3_OFF digitalWrite(LED3, HIGH);
#define LED4_ON  digitalWrite(LED4, LOW);
#define LED4_OFF digitalWrite(LED4, HIGH);
#endif

#define DRV_ENABLE_ON  digitalWrite(DRV_ENABLE, HIGH);
#define DRV_ENABLE_OFF digitalWrite(DRV_ENABLE, LOW);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
