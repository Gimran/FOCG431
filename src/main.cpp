#include <Arduino.h>
#include <SimpleFOC.h>

#include <SimpleFOCDrivers.h>
#include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"
#include <SPI.h>
#include <drv832x.h>
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

#define RS485_DIR_PIN PB1
#define COM_OW Serial3

// SPI пины
#define DRV_MOSI PB5
#define DRV_MISO PB4
#define DRV_SCK  PB3
#define DRV_CS   PA4  // Укажи свой CS пин

// Другие пины
#define DRV_CAL    PB8
#define DRV_NFAULT PB6
#define DRV_ENABLE PB7


BLDCMotor motor = BLDCMotor(7, 2.3, 220, 0.00086);

// BLDCDriver6PWM driver = BLDCDriver6PWM(PA10, PB15, PA9, PB14, PA8, PB13, PB7);
DRV832xDriver6PWM driver = DRV832xDriver6PWM(PA10, PB15, PA9, PB14, PA8, PB13, DRV_CS, false, DRV_ENABLE, DRV_NFAULT);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.005, 40, PA0, PA1, PA2);

MXLEMMINGObserverSensor observer = MXLEMMINGObserverSensor(motor); // observer sensor instance

HardwareSerial Serial3(PB11, PB10);
SPIClass SPI_3(DRV_MOSI, DRV_MISO, DRV_SCK);

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

void setup()
{

  pinMode(RS485_DIR_PIN, OUTPUT);
  digitalWrite(RS485_DIR_PIN, LOW);
  Serial3.begin(921600);
  COM_OW.println("start");

  // DRV8323
  pinMode(DRV_ENABLE, OUTPUT);
  digitalWrite(DRV_ENABLE, HIGH);
  SPI_3.begin();

  command.add('A', doAnalog, "analog read A0-A4");

  SimpleFOCDebug::enable(&COM_OW);



  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 20000;
  driver.init(&SPI_3);
  driver.setCurrentSenseGain(DRV832x_CSAGain::CSAGain_20);

  driver.enable();

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
  motor.monitor();
  command.run();
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}