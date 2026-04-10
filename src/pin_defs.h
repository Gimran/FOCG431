
#define PCB_5410
// #define PCB_3200

#if defined(PCB_3200)
#define RS485_DIR_PIN PB1
#define TEMP_SENSOR PA5
#define UART3_TX PB10
#define UART3_RX PB11
#elif defined(PCB_5410)
#define RS485_DIR_PIN PB9
#define RS485_DIR_PORT GPIOB
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

#define SERIES_RESISTOR 10000.0f     // 10k pulldown
#define NOMINAL_RESISTANCE 100000.0f // 100k при 25 градусах
#define NOMINAL_TEMPERATURE 298.15f  // 25 градусов в Кельвинах
#define B_COEFFICIENT 3950.0f        // Типовой B-коэффициент для NTC 100k
#define ADC_MAX 4095.0f              // Для 12-битного АЦП

// #define VOLTAGE_SENSOR VIN_VOLTAGE // Замени на свой пин
#define R1_DIV 100000.0f   // Верхний резистор (к измеряемому напряжению)
#define R2_DIV 6800.0f     // Нижний резистор (к земле)
#define V_REF 3.3f         // Опорное напряжение (напряжение питания МК)