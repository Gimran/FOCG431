#include "main.h"
#include <Arduino.h>
#include <SimpleFOC.h>

#include <SimpleFOCDrivers.h>
#include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"
#include <SPI.h>
#include <drv832x.h>
#include <drv8323rs.h>
// #include "./lib/drv832x.h"
// #include ""

// #include <SimpleFOCDrivers.h>
// #include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"

// Applicable Voltage: DC 7.4-16V
// Rated Voltage: 12V
// Rated Current: 0.4A
// Maximum Current: 1A
// Rated Power: 12W
// Rated Speed: 2600 RPM @ 12V
// Torque: Approximately 300g.cm/0.03Nm
// Number of Slots and Poles: 12N/14P
// Number of Pole Pairs: 7
// Winding Resistance: 5.1Ω
// Phase Resistance: 2.3Ω
// KV Rating: 220KV
// Winding Inductance: 2.8mH
// Phase Inductance: 0.86mH
// Magnetic Flux: 0.0035Wb
// Winding Method: Delta Connection

// https://docs.simplefoc.com/sensorless_foc_nucleo_example




BLDCMotor motor = BLDCMotor(7, 2.3, 220, 0.00086);

// BLDCDriver6PWM driver = BLDCDriver6PWM(PA10, PB15, PA9, PB14, PA8, PB13, PB7);
DRV832xDriver6PWM driver = DRV832xDriver6PWM(PA10, PB15, PA9, PB14, PA8, PB13, DRV_CS, false, DRV_ENABLE, DRV_NFAULT);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.005, 40, PA0, PA1, PA2);

MXLEMMINGObserverSensor observer = MXLEMMINGObserverSensor(motor); // observer sensor instance

HardwareSerial Serial3(PB11, PB10);
SPIClass SPI_3(DRV_MOSI, DRV_MISO, DRV_SCK);
DRV8323_VARS_t gDrv8323 = DRV8323_DEFAULTS;

Commander command = Commander(COM_OW);


// Commander command = Commander(Serial3);
void doMotor(char* cmd){ command.motor(&motor, cmd);}

// get analog input 
void doAnalog(char* cmd){ 
    if (cmd[0] == '0') Serial.println("check_serial");
    else if (cmd[0] == '1') Serial.println("check_serial1");
}
// put function declarations here:
// int myFunction(int, int);

// ----------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------
// DRV830x SPI Input Data bit definitions:
struct  DRV830x_SPI_WRITE_WORD_BITS {       // bit      description
    uint16_t DATA:11;                       // 10:0     FIFO reset
    uint16_t ADDRESS:4;                     // 14:11    Enhancement enable
    uint16_t R_W:1;                         // 15       R/W
};

union DRV830x_SPI_WRITE_WORD_REG {
    uint16_t                           all;
    struct DRV830x_SPI_WRITE_WORD_BITS bit;
};



uint16_t readDRVReg(uint8_t addr) {
  digitalWrite(DRV_CS, LOW);
  SPI_3.beginTransaction(SPISettings(10000, MSBFIRST, SPI_MODE1));
  // Бит 15 = 1 (Чтение), биты 14-11 = адрес
  uint16_t tx_data = 0x8000 | (addr << 11);
  uint16_t rx_data = SPI_3.transfer16(tx_data);
  SPI_3.endTransaction();
  digitalWrite(DRV_CS, HIGH);
  return rx_data; //& 0x07FF; // Значения лежат в младших 11 битах
}

uint16_t SPI_Driver(DRV8323_VARS_t *v, uint16_t data)
{
    uint8_t highByte = (uint8_t)((data >> 8) & 0x00FF);
    uint8_t lowByte  = (uint8_t)(data & 0x00FF);
    uint16_t returnValue;
    // digitalWrite(DRV_CS, LOW);

    //USCI_A0 TX buffer ready?
    // while (!USCI_B_SPI_getInterruptStatus(USCI_B0_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    //Transmit Data to slave
    // USCI_B_SPI_transmitData(USCI_B0_BASE, highByte);
    SPI_3.beginTransaction(SPISettings(10000, MSBFIRST, SPI_MODE1));
    returnValue = SPI_3.transfer(highByte);
    returnValue |= SPI_3.transfer(lowByte);



    // //USCI_A0 RX buffer ready?
    // while (!USCI_B_SPI_getInterruptStatus(USCI_B0_BASE, USCI_B_SPI_RECEIVE_INTERRUPT));
    // returnValue = (USCI_B_SPI_receiveData(USCI_B0_BASE) << 8) &0xFF00;

    // //USCI_A0 TX buffer ready?
    // while (!USCI_B_SPI_getInterruptStatus(USCI_B0_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT));
    // //Transmit Data to slave
    // USCI_B_SPI_transmitData(USCI_B0_BASE, lowByte);

    // //USCI_A0 RX buffer ready?
    // while (!USCI_B_SPI_getInterruptStatus(USCI_B0_BASE, USCI_B_SPI_RECEIVE_INTERRUPT));
    // returnValue |= USCI_B_SPI_receiveData(USCI_B0_BASE) &0x00FF;

    return returnValue;
}


void DRV8323_SPI_Read(DRV8323_VARS_t *v, uint16_t address)
{
    union DRV830x_SPI_WRITE_WORD_REG w;
    uint16_t * cntrlReg;

    cntrlReg = (uint16_t*)&(v->Fault_Status_1);
    w.bit.R_W = READ;
    w.bit.ADDRESS = address;
    w.bit.DATA = 0;

    // Enable CS; SPI transfer; Disable CS
    digitalWrite(DRV_CS, LOW);// GPIO_setOutputLowOnPin(v->ScsPort, v->ScsPin);
    
    cntrlReg[address] = SPI_Driver(v, w.all);
    digitalWrite(DRV_CS, HIGH);// GPIO_setOutputHighOnPin(v->ScsPort, v->ScsPin);
}


void printDrv8323Regs() {
  Serial3.println("--- DRV8323 gDrv8323 Registers ---");
  Serial3.print("Fault_Status_1: 0x");  Serial3.println(gDrv8323.Fault_Status_1.all,  HEX);
  Serial3.print("VGS_Status_2: 0x");    Serial3.println(gDrv8323.VGS_Status_2.all,    HEX);
  Serial3.print("Driver_Control: 0x");  Serial3.println(gDrv8323.Driver_Control.all,  HEX);
  Serial3.print("Gate_Drive_HS: 0x");   Serial3.println(gDrv8323.Gate_Drive_HS.all,   HEX);
  Serial3.print("Gate_Drive_LS: 0x");   Serial3.println(gDrv8323.Gate_Drive_LS.all,   HEX);
  Serial3.print("OCP_Control: 0x");     Serial3.println(gDrv8323.OCP_Control.all,     HEX);
  Serial3.print("CSA_Control: 0x");     Serial3.println(gDrv8323.CSA_Control.all,     HEX);
  Serial3.print("fault: 0x");           Serial3.println(gDrv8323.fault,               HEX);
  Serial3.print("ScsPort: 0x");         Serial3.println(gDrv8323.ScsPort,             HEX);
  Serial3.print("ScsPin: 0x");          Serial3.println(gDrv8323.ScsPin,              HEX);
  Serial3.println("----------------------------------");

}


void setup()
{

  pinMode(RS485_DIR_PIN, OUTPUT);
  digitalWrite(RS485_DIR_PIN, LOW);
  Serial3.begin(921600);
  COM_OW.println("start");

  // DRV8323
  pinMode(DRV_ENABLE, OUTPUT);
  digitalWrite(DRV_ENABLE, HIGH);
  // pinMode(DRV_NFAULT, INPUT);
  pinMode(DRV_CS, OUTPUT);
  // digitalWrite(DRV_CS, HIGH);
  SPI_3.begin();

  command.add('A', doAnalog, "analog read A0-A4");

  SimpleFOCDebug::enable(&COM_OW);




  // driver config
  driver.enable();
  driver.init(&SPI_3);
  // driver.init();

    for(uint16_t i= 0; i < DRV8323_CSA_CNTRL_ADDR + 1; i++)
    {
        DRV8323_SPI_Read(&gDrv8323, i);
        HAL_Delay(5);
    }

    // Serial3.print("CSA_Control_REG:%X", gDrv8323.CSA_Control.all);
    
  //   Serial3.println("--- DRV8323 Default Registers ---");
  // // Чтение регистров с 0x0 по 0x6
  // for (uint8_t i = 0; i <= 10; i++) {
  //   uint16_t val = readDRVReg(i);
  //   Serial3.print("Reg 0x");
  //   Serial3.print(i, HEX);
  //   Serial3.print(": 0x");
  //   if (val < 0x10) Serial3.print("0"); // Ведущие нули
  //   if (val < 0x100) Serial3.print("0");
  //   Serial3.println(val, BIN);
  // }
  // Serial3.println("---------------------------------");
  printDrv8323Regs();

  /*
  --- DRV8323 Default Registers ---
Reg 0x0: 0x00000000000
Reg 0x1: 0x00000000000
Reg 0x2: 0x00000000000
Reg 0x3: 0x01111111111
Reg 0x4: 0x11111111111
Reg 0x5: 0x00101011001
Reg 0x6: 0x01010000011
Reg 0x7: 0x00000000000

  */


  //   driver.setRegistersLocked(false);

  // // // Токи затвора для BSZ014NE2LS5IF
  driver.setHighSideChargeCurrent(DRV832x_IDRIVEP::IDRIVEP_120mA);

  //     for(uint16_t i= 0; i < DRV8323_CSA_CNTRL_ADDR + 1; i++)
  //   {
  //       DRV8323_SPI_Read(&gDrv8323, i);
  //       HAL_Delay(5);
  //   }

  // printDrv8323Regs();
  driver.setHighSideDischargeCurrent(DRV832x_IDRIVEN::IDRIVEN_240mA);
  driver.setLowSideChargeCurrent(DRV832x_IDRIVEP::IDRIVEP_120mA);
  driver.setLowSideDischargeCurrent(DRV832x_IDRIVEN::IDRIVEN_240mA);

  driver.setDeadTime(DRV832x_DEADTIME::DEADTIME_200ns);
  driver.setVDSLevel(DRV832x_VDS_LVL::VDS_LVL_260mV);

  // Настройка токовых усилителей под VREF = 3.3V
  driver.setCurrentSenseBidirectionnal(true); // Ноль тока = VREF/2 = 1.65В
  driver.setCurrentSenseOvercurrentSensitivity(DRV832x_CS_VSEN_LVL::CSAGain_250mV); // Защита 1.25А
  driver.setCurrentSenseGain(DRV832x_CSAGain::CSAGain_40); // Усиление 40

    // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 20000;


  current_sense.linkDriver(&driver);
  current_sense.init();

  motor.linkSensor(&observer);
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity_openloop;
  motor.torque_controller = TorqueControlType::foc_current;

  motor.linkCurrentSense(&current_sense);
  motor.useMonitoring(COM_OW);

    // skip the sensor alignment
  motor.sensor_direction = Direction::CW;
  motor.zero_electric_angle = 0;

  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_CURR_Q | _MON_VEL | _MON_ANGLE;
  motor.monitor_downsample = 50; // default 10

  motor.current_limit = 0.5;  //amp

  motor.init();


  motor.initFOC();

  // subscribe motor to the commander
  command.add('M', doMotor, "motor");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  COM_OW.println("Motor ready.");
  // LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
  Serial3.print("System Clock: ");
Serial3.print(SystemCoreClock / 1000000);
Serial3.println(" MHz");



  _delay(1000);
}

void loop() {
  motor.move();
  motor.loopFOC();
  // motor.monitor();
  command.run();
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}