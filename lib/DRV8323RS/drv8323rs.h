#ifndef DRV8323RS
#define DRV8323RS

#include "Arduino.h"
// #include <SPI.h>
#include <drv8323rs_defs.h>

// DRV830x SPI Input Data bit definitions:
struct DRV830x_SPI_WRITE_WORD_BITS
{                       // bit      description
  uint16_t DATA : 11;   // 10:0     FIFO reset
  uint16_t ADDRESS : 4; // 14:11    Enhancement enable
  uint16_t R_W : 1;     // 15       R/W
};

union DRV830x_SPI_WRITE_WORD_REG
{
  uint16_t all;
  struct DRV830x_SPI_WRITE_WORD_BITS bit;
};


#endif