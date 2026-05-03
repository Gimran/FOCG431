#include "drv_utils.h"
#include "drv832x.h"


// uint16_t SPI_Driver(SPIClass SPI_3, DRV8323_VARS_t *v, uint16_t data)
// {
//   uint8_t highByte = (uint8_t)((data >> 8) & 0x00FF);
//   uint8_t lowByte = (uint8_t)(data & 0x00FF);
//   uint16_t returnValue;
//   SPI_3.beginTransaction(SPISettings(10000, MSBFIRST, SPI_MODE1));
//   returnValue = SPI_3.transfer16(data);
//   SPI_3.endTransaction();
//   return returnValue;
// }
// void DRV8323_SPI_Read(DRV8323_VARS_t *v, uint16_t address)
// {
//   union DRV830x_SPI_WRITE_WORD_REG w;
//   uint16_t *cntrlReg;

//   cntrlReg = (uint16_t *)&(v->Fault_Status_1);
//   w.bit.R_W = READ;
//   w.bit.ADDRESS = address;
//   w.bit.DATA = 0;

//   // Enable CS; SPI transfer; Disable CS
//   digitalWrite(DRV_CS, LOW);

//   cntrlReg[address] = SPI_Driver(v, w.all);

//   digitalWrite(DRV_CS, HIGH);
// }
// void DRV8323_SPI_Write(DRV8323_VARS_t *v, uint16_t address)
// {
//   union DRV830x_SPI_WRITE_WORD_REG w;
//   uint16_t *cntrlReg;

//   cntrlReg = (uint16_t *)&(v->Fault_Status_1);
//   w.bit.R_W = WRITE;
//   w.bit.ADDRESS = address;
//   w.bit.DATA = cntrlReg[address];

//   digitalWrite(DRV_CS, LOW);
//   SPI_Driver(v, w.all);
//   digitalWrite(DRV_CS, HIGH);
// }


void printDrv8323Regs(DRV8323_VARS_t gDrv8323)
{
  Serial3.println("--- DRV8323 gDrv8323 Registers ---");
  Serial3.print("Fault_Status_1: 0x");
  Serial3.println(gDrv8323.Fault_Status_1.all, HEX);
  Serial3.print("VGS_Status_2: 0x");
  Serial3.println(gDrv8323.VGS_Status_2.all, HEX);
  Serial3.print("Driver_Control: 0x");
  Serial3.println(gDrv8323.Driver_Control.all, HEX);
  Serial3.print("Gate_Drive_HS: 0x");
  Serial3.println(gDrv8323.Gate_Drive_HS.all, HEX);
  Serial3.print("Gate_Drive_LS: 0x");
  Serial3.println(gDrv8323.Gate_Drive_LS.all, HEX);
  Serial3.print("OCP_Control: 0x");
  Serial3.println(gDrv8323.OCP_Control.all, HEX);
  Serial3.print("CSA_Control: 0x");
  Serial3.println(gDrv8323.CSA_Control.all, HEX);
  Serial3.print("fault: 0x");
  Serial3.println(gDrv8323.fault, HEX);
  Serial3.print("ScsPort: 0x");
  Serial3.println(gDrv8323.ScsPort, HEX);
  Serial3.print("ScsPin: 0x");
  Serial3.println(gDrv8323.ScsPin, HEX);
  Serial3.println("----------------------------------");
}

// void DRV8323_SPI_Read(DRV8323_VARS_t *v, uint16_t address)
// {
//   union DRV830x_SPI_WRITE_WORD_REG w;
//   uint16_t *cntrlReg;

//   cntrlReg = (uint16_t *)&(v->Fault_Status_1);
//   w.bit.R_W = READ;
//   w.bit.ADDRESS = address;
//   w.bit.DATA = 0;

//   // Enable CS; SPI transfer; Disable CS
//   digitalWrite(DRV_CS, LOW);

//   cntrlReg[address] = SPI_Driver(v, w.all);

//   digitalWrite(DRV_CS, HIGH);
// }
// void DRV8323_SPI_Write(DRV8323_VARS_t *v, uint16_t address)
// {
//   union DRV830x_SPI_WRITE_WORD_REG w;
//   uint16_t *cntrlReg;

//   cntrlReg = (uint16_t *)&(v->Fault_Status_1);
//   w.bit.R_W = WRITE;
//   w.bit.ADDRESS = address;
//   w.bit.DATA = cntrlReg[address];

//   digitalWrite(DRV_CS, LOW);
//   SPI_Driver(v, w.all);
//   digitalWrite(DRV_CS, HIGH);
// }

// void checkFault(DRV8323_VARS_t gDrv8323)
// {
//   if (!digitalRead(DRV_NFAULT))
//   {
//     char buffer[33];
//     char buffer2[33];
//     printf(" FAULT INTERRUPT!\r\n");
//     // digitalWrite(DRV_ENABLE, HIGH);
//     _delay(10);
//     DRV8323_SPI_Read(&gDrv8323, DRV8323_FAULT_STATUS_1_ADDR);
//     _delay(10);
//     DRV8323_SPI_Read(&gDrv8323, DRV8323_VGS_STATUS_2_ADDR);
//     // itoa(gDrv8323.Fault_Status_1.all, buffer, 2);
//     // printf("Fault_Status_1: %s\r\n", buffer);
//     printf("Fault_Status_1: 0x%X\r\n", gDrv8323.Fault_Status_1.all);
//     printf("VGS_Status_2: 0x%X\r\n", gDrv8323.VGS_Status_2.all);
//     // itoa(gDrv8323.VGS_Status_2.all, buffer2, 2);
//     // printf("VGS_Status_2: %s\r\n", buffer2);
//     printf("VDS_LA: \t VDS_HA: \t VDS_LB: \t VDS_HB: \t VDS_LC: \t VDS_HC:\r\n");
//     printf("%d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\r\n",
//              gDrv8323.Fault_Status_1.bit.VDS_LA, gDrv8323.Fault_Status_1.bit.VDS_HA,
//              gDrv8323.Fault_Status_1.bit.VDS_LB, gDrv8323.Fault_Status_1.bit.VDS_HB,
//              gDrv8323.Fault_Status_1.bit.VDS_LC, gDrv8323.Fault_Status_1.bit.VDS_HC);
    
//     printf("OTSD: %d\r\n", gDrv8323.Fault_Status_1.bit.OTSD);
//     printf("UVLO: %d\r\n", gDrv8323.Fault_Status_1.bit.UVLO);
//     printf("GDF: %d\r\n", gDrv8323.Fault_Status_1.bit.GDF);
//     printf("VDS_OCP: %d\r\n", gDrv8323.Fault_Status_1.bit.VDS_OCP);
//   }
// }