/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define PCB_5410
// #define PCB_3200

#if defined(PCB_3200)
#define RS485_DIR_PIN PB1
#define TEMP_SENSOR PA5
#define UART3_TX PB10
#define UART3_RX PB11
#elif defined(PCB_5410)
#define RS485_DIR_PIN PB9
#define TEMP_SENSOR PA3
#define VIN_VOLTAGE PB12
#define UART3_TX PB10
#define UART3_RX PB11
#define UART1_TX PB6
#define UART1_RX PB7
#endif

#if defined(PCB_5410)
#define ENC_SPI_NSS PA4
#define ENC_SPI_SCK PA5
#define ENC_SPI_MISO PA6
#define ENC_SPI_MOSI PA7

#define LED1 PB0
#define LED2 PB1
#define LED3 PB2
#define LED4 PC13
#endif

#define UART_COM Serial3
#define UART_ENC Serial1

// PWM pins
#define DRV_PHASE_A_H PA10
#define DRV_PHASE_A_L PB15
#define DRV_PHASE_B_H PA9
#define DRV_PHASE_B_L PB14
#define DRV_PHASE_C_H PA8
#define DRV_PHASE_C_L PB13
#if defined(PCB_3200)
#define DRV_ENABLE    PB7
#elif defined(PCB_5410)
#define DRV_ENABLE    PC14
#endif
// end PWM pins

// SPI pins
#define DRV_MOSI PB5
#define DRV_MISO PB4
#define DRV_SCK  PB3
#define DRV_CS   PA15
// end SPI pins

// DRV pins
#if defined(PCB_3200)
#define DRV_CAL_AMP   PB8
#define DRV_NFAULT PB6
#elif defined(PCB_5410)
#define DRV_CAL_AMP   PC15
#define DRV_NFAULT PB8
#endif

#define DRV_PHASE_A_CUR PA0
#define DRV_PHASE_B_CUR PA1
#define DRV_PHASE_C_CUR PA2



#define CAN_SHDN NC
#define CAN_ENABLE NC
#define CAN_RX PA11
#define CAN_TX PA12

#ifndef READ
#define READ                1
#endif
#ifndef WRITE
#define WRITE               0
#endif



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
