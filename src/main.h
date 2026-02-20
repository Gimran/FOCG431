/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define RS485_DIR_PIN PB1
#define COM_OW Serial3

// SPI пины
#define DRV_MOSI PB5
#define DRV_MISO PB4
#define DRV_SCK  PB3
#define DRV_CS   PA15  // Укажи свой CS пин

// Другие пины
#define DRV_CAL    PB8
#define DRV_NFAULT PB6
#define DRV_ENABLE PB7


// #ifndef HIGH
// #define HIGH                1
// #endif
// #ifndef LOW
// #define LOW                 0
// #endif

#ifndef READ
#define READ                1
#endif
#ifndef WRITE
#define WRITE               0
#endif


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
