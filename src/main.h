/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define UART_COM Serial3
#define UART_ENC Serial1

//TODO - move to gpio lib
#define LED1_ON  digitalWrite(LED1, LOW);
#define LED1_OFF digitalWrite(LED1, HIGH);
#define LED2_ON  digitalWrite(LED2, LOW);
#define LED2_OFF digitalWrite(LED2, HIGH);
#define LED3_ON  digitalWrite(LED3, LOW);
#define LED3_OFF digitalWrite(LED3, HIGH);
#define LED4_ON  digitalWrite(LED4, LOW);
#define LED4_OFF digitalWrite(LED4, HIGH);

#define DRV_ENABLE_ON  digitalWrite(DRV_ENABLE, HIGH);
#define DRV_ENABLE_OFF digitalWrite(DRV_ENABLE, LOW);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
