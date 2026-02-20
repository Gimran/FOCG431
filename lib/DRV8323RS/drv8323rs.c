#include <drv8323rs.h>
// #include <SPI.h>
// #include "main.h"
// ----------------------------------------------------------------------
// Defines
// ----------------------------------------------------------------------

// #define RS485_DIR_PIN PB1
// #define COM_OW Serial3

// // SPI пины
// #define DRV_MOSI PB5
// #define DRV_MISO PB4
// #define DRV_SCK  PB3
// #define DRV_CS   PA15  // Укажи свой CS пин

// // Другие пины
// #define DRV_CAL    PB8
// #define DRV_NFAULT PB6
// #define DRV_ENABLE PB7

// SPIClass SPI_3(DRV_MOSI, DRV_MISO, DRV_SCK);



// // ----------------------------------------------------------------------
// // Global Variables
// // ----------------------------------------------------------------------
// // DRV830x SPI Input Data bit definitions:
// struct  DRV830x_SPI_WRITE_WORD_BITS {       // bit      description
//     uint16_t DATA:11;                       // 10:0     FIFO reset
//     uint16_t ADDRESS:4;                     // 14:11    Enhancement enable
//     uint16_t R_W:1;                         // 15       R/W
// };

// union DRV830x_SPI_WRITE_WORD_REG {
//     uint16_t                           all;
//     struct DRV830x_SPI_WRITE_WORD_BITS bit;
// };

// void DRV8323_SPI_Read(DRV8323_VARS_t *v, uint16_t address)
// {
//     union DRV830x_SPI_WRITE_WORD_REG w;
//     uint16_t * cntrlReg;

//     cntrlReg = (uint16_t*)&(v->Fault_Status_1);
//     w.bit.R_W = READ;
//     w.bit.ADDRESS = address;
//     w.bit.DATA = 0;

//     // Enable CS; SPI transfer; Disable CS
//     // GPIO_setOutputLowOnPin(v->ScsPort, v->ScsPin);
//     // cntrlReg[address] = SPI_Driver(v, w.all);
//     // GPIO_setOutputHighOnPin(v->ScsPort, v->ScsPin);
// }

// uint16_t readDRVReg(uint8_t addr) {
//   digitalWrite(DRV_CS, LOW);
//   SPI_3.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
//   // Бит 15 = 1 (Чтение), биты 14-11 = адрес
//   uint16_t tx_data = 0x8000 | (addr << 11);
//   uint16_t rx_data = SPI_3.transfer16(tx_data);
//   SPI_3.endTransaction();
//   digitalWrite(DRV_CS, HIGH);
//   return rx_data & 0x07FF; // Значения лежат в младших 11 битах
// }