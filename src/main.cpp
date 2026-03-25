#include "main.h"
#include <Arduino.h>
#include <SimpleFOC.h>

#include <SimpleFOCDrivers.h>
#include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
#include "utilities/trapezoids/TrapezoidalPlanner.h"
#include <SPI.h>
#include <SimpleCANio.h>
#include "comms/can/CANCommander.h" //https://github.com/simplefoc/Arduino-FOC-drivers/blob/feat_can_commander/examples/comms/can/can_example/can_example.ino
// warning error https://community.simplefoc.com/t/stm32h750-low-side-current-measurement/7342/2
// #include <drv832x.h>
#include <drv8323rs.h>
#include <LibPrintf.h>
#include "motor_settings.h"

#define CAN_ID 228
#define USE_UART_COMMANDER
// #define USE_CAN_COMMANDER



void printDrv8323Regs();
uint16_t SPI_Driver(DRV8323_VARS_t *v, uint16_t data);
void DRV8323_SPI_Read(DRV8323_VARS_t *v, uint16_t address);
void DRV8323_SPI_Write(DRV8323_VARS_t *v, uint16_t address);
void checkFault();
void rampACC(float target_speed, float acceleration_rate);
void doRegisters(char *cmd);
void doDriver(char *cmd);
void GPIO_INIT();
void drvReadAllRegs();
void setup_DRV_registers (uint8_t mode);
void fault_ISR();
void DRV_enable_ISR();

/**
     BLDCMotor class constructor
     @param pp  pole pairs number
     @param R   motor phase resistance - [Ohm]
     @param KV  motor KV rating (1/K_bemf) - rpm/V
     @param Lq  motor q-axis inductance - [H]
     @param Ld  motor d-axis inductance - [H]
     */
BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, KV_RATING, L_Q);

BLDCDriver6PWM driverBase = BLDCDriver6PWM(DRV_PHASE_A_H, DRV_PHASE_A_L,
                                           DRV_PHASE_B_H, DRV_PHASE_B_L, 
                                           DRV_PHASE_C_H, DRV_PHASE_C_L, 
                                           DRV_ENABLE);
    /**
      LowsideCurrentSense class constructor
      @param shunt_resistor shunt resistor value
      @param gain current-sense op-amp gain */
LowsideCurrentSense current_sense = LowsideCurrentSense(SHUNT_OM, 
                                                        GAIN, 
                                                        DRV_PHASE_A_CUR, 
                                                        DRV_PHASE_B_CUR, 
                                                        DRV_PHASE_C_CUR);
MXLEMMINGObserverSensor observer = MXLEMMINGObserverSensor(motor); // observer sensor instance

// HardwareSerial Serial1(UART1_RX, UART1_TX);
HardwareSerial Serial3(UART3_RX, UART3_TX);
SPIClass SPI_3(DRV_MOSI, DRV_MISO, DRV_SCK);

DRV8323_VARS_t gDrv8323 = DRV8323_DEFAULTS;


#ifdef USE_UART_COMMANDER
Commander command = Commander(UART_COM);
#elifdef USE_CAN_COMMANDER
CANio can = CANio(CAN_RX, CAN_TX, CAN_SHDN, CAN_ENABLE); // <- create SimpleCAN object
CANCommander commander(can, CAN_ID);
#endif

#ifdef USE_UART_COMMANDER
void doMotor(char *cmd) { command.motor(&motor, cmd); }
#endif

void onTarget(char* cmd){ 
    // get the target velocity in RPM
    float target_velocity_RPM = atof(cmd);
    // set the target velocity in radians per second
    motor.target = target_velocity_RPM * _2PI/60;
}
//!SECTION
void setup()  //SECTION - setup
{
  printf_init(UART_COM);
  SimpleFOC_CORDIC_Config();      // initialize the CORDIC
  // SPI_3.setSSEL(DRV_CS);
  SPI_3.begin();
  Serial3.begin(921600);
  UART_COM.println("start");
  GPIO_INIT();
  digitalWrite(LED1, LOW);

  // digitalWrite(DRV_ENABLE, HIGH);
  // digitalWrite(DRV_CS, LOW);
  
  // digitalWrite(DRV_CAL_AMP, HIGH);
  // _delay(50);
  // digitalWrite(DRV_CAL_AMP, LOW);
  // _delay(50);
  
  //ANCHOR - command setup
  #ifdef USE_UART_COMMANDER
  printf("\033[2J\033[H");
  printf("%s / %s\r\n", __DATE__, __TIME__);
  printf("COM_init\r\n");
  LED1_ON
  command.add('M', doMotor, "motor");
  command.add('R', doRegisters, "change DRV8323 registers");
  command.add('V', onTarget, "velocity in RPM");
  command.add('D', doDriver, "change driver settings");
  #elifdef USE_CAN_COMMANDER
  printf("CAN_init\r\n");
  commander.baudrate = 1000000; // Set CAN baudrate to 1 Mbps
  commander.init();
  printf("CAN_init_COMPL\r\n");
  commander.addMotor(&motor);
  printf("CAN_addmotor_COMPL\r\n");
  #endif

  // drvReadAllRegs();
  // printDrv8323Regs();

  


  // ANCHOR - driver setup
  driverBase.voltage_power_supply = OPERATION_VOLTAGE;
  driverBase.pwm_frequency = PWM_FREQ;

  driverBase.enable();
  _delay(50);
  setup_DRV_registers(1);
  _delay(50);
  drvReadAllRegs();
  printDrv8323Regs();
  // driverBase.disable();
  _delay(50);
  driverBase.init();
  // digitalWrite(DRV_ENABLE, HIGH);
  _delay(50);
  // checkFault();
  
  


  current_sense.linkDriver(&driverBase);
  current_sense.init();
  
  // power supply voltage [V]
  
  // driverBase.dead_zone = 0.05f; // 5% dead time for both high and low side


  motor.voltage_sensor_align = 1.0;
  
  
  // digitalWrite(DRV_ENABLE, HIGH);
  // digitalWrite(DRV_PHASE_A_L, LOW);
  // digitalWrite(DRV_PHASE_C_L, HIGH);

    PhaseCurrent_s currents = current_sense.getPhaseCurrents();
    printf("Currents: Ia=%.3f A, Ib=%.3f A, Ic=%.3f A\r\n",
                         currents.a, currents.b, currents.c);
  
  
  
  motor.linkDriver(&driverBase);
  motor.linkSensor(&observer);
  // /*

  motor.controller = MotionControlType::velocity_openloop;
  // motor.controller = MotionControlType::angle_openloop;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.linkCurrentSense(&current_sense);
  motor.useMonitoring(UART_COM);

  // skip the sensor alignment
  motor.sensor_direction = Direction::CW;
  motor.zero_electric_angle = 0;
  // motor.velocity_limit = 500;     // rpm

  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_CURR_Q | _MON_VEL | _MON_ANGLE;
  // motor.monitor_downsample = 100; // default 10

  motor.current_limit = MAX_CURRENT; // amp

  motor.init();
  int motor_ready_flag = motor.initFOC();
  if(motor_ready_flag){UART_COM.println("Motor ready.");}

  // motor.velocity_limit = 500;     // rpm
  // motor.PID_velocity.limit = 0.3; // amp
  // motor.PID_velocity.output_ramp = 10; //!< Maximum speed of change of the output value
  // motor.LPF_velocity.Tf = 0.05;  //!< Low pass filter time constant
  #ifdef USE_CAN_COMMANDER
  commander.echo = true; // Echo received commands back to the sender
  #endif

  _delay(100);
  
} //!SECTION

// int32_t downsample = 100;    // depending on your MCU's speed, you might want a value between 5 and 50...
// int32_t downsample_cnt = 0;

void loop() //ANCHOR - LOOP
{
  // if (downsample > 0 && --downsample_cnt <= 0) {
  //   motor.target = trapezoidal.run();
  //   downsample_cnt = downsample;
  // }

  motor.loopFOC();
  motor.move();
  #ifdef USE_UART_COMMANDER
  command.run();
  #elifdef USE_CAN_COMMANDER
  commander.run();
  #endif
  // motor.monitor();

  // checkFault();

  // PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  //   printf("Currents: Ia=%.3f A, Ib=%.3f A, Ic=%.3f A\r\n",
  //                        currents.a, currents.b, currents.c);
    
  //   delay(100);
}

void rampACC(float target_speed, float acceleration_rate)
{
  static float current_target = 0.0;      // Переменная для хранения текущего шага
  static long last_timestamp = _micros(); // Таймер
  // Рассчитываем время, прошедшее с прошлого прохода
  long now = _micros();
  float dt = (now - last_timestamp) * 1e-6f; // Переводим микросекунды в секунды
  last_timestamp = now;

  // Увеличиваем текущую цель плавно во времени
  if (current_target < target_speed)
  {
    current_target += acceleration_rate * dt;
    if (current_target > target_speed)
      current_target = target_speed;
  }
  else if (current_target > target_speed)
  {
    current_target -= acceleration_rate * dt;
    if (current_target < target_speed)
      current_target = target_speed;
  }

  motor.target = current_target;
}

void checkFault()
{
  if (!digitalRead(DRV_NFAULT))
  {
    char buffer[33];
    char buffer2[33];
    Serial3.println(" FAULT INTERRUPT!");
    // digitalWrite(DRV_ENABLE, HIGH);
    _delay(10);
    DRV8323_SPI_Read(&gDrv8323, DRV8323_FAULT_STATUS_1_ADDR);
    _delay(10);
    DRV8323_SPI_Read(&gDrv8323, DRV8323_VGS_STATUS_2_ADDR);
    // itoa(gDrv8323.Fault_Status_1.all, buffer, 2);
    // printf("Fault_Status_1: %s\r\n", buffer);
    printf("Fault_Status_1: 0x%X\r\n", gDrv8323.Fault_Status_1.all);
    printf("VGS_Status_2: 0x%X\r\n", gDrv8323.VGS_Status_2.all);
    // itoa(gDrv8323.VGS_Status_2.all, buffer2, 2);
    // printf("VGS_Status_2: %s\r\n", buffer2);
    printf("VDS_LA: \t VDS_HA: \t VDS_LB: \t VDS_HB: \t VDS_LC: \t VDS_HC:\r\n");
    printf("%d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\r\n",
             gDrv8323.Fault_Status_1.bit.VDS_LA, gDrv8323.Fault_Status_1.bit.VDS_HA,
             gDrv8323.Fault_Status_1.bit.VDS_LB, gDrv8323.Fault_Status_1.bit.VDS_HB,
             gDrv8323.Fault_Status_1.bit.VDS_LC, gDrv8323.Fault_Status_1.bit.VDS_HC);
    
    printf("OTSD: %d\r\n", gDrv8323.Fault_Status_1.bit.OTSD);
    printf("UVLO: %d\r\n", gDrv8323.Fault_Status_1.bit.UVLO);
    printf("GDF: %d\r\n", gDrv8323.Fault_Status_1.bit.GDF);
    printf("VDS_OCP: %d\r\n", gDrv8323.Fault_Status_1.bit.VDS_OCP);
    // printf("FAULT: %d\r\n", gDrv8323.Fault_Status_1.bit.FAULT);
    // digitalWrite(DRV_ENABLE, LOW); // Отключаем драйвер для безопасности
    // delay(1000);
  }
}


uint16_t SPI_Driver(DRV8323_VARS_t *v, uint16_t data)
{
  uint8_t highByte = (uint8_t)((data >> 8) & 0x00FF);
  uint8_t lowByte = (uint8_t)(data & 0x00FF);
  uint16_t returnValue;
  SPI_3.beginTransaction(SPISettings(10000, MSBFIRST, SPI_MODE1));
  returnValue = SPI_3.transfer16(data);
  SPI_3.endTransaction();
  return returnValue;
}
void DRV8323_SPI_Read(DRV8323_VARS_t *v, uint16_t address)
{
  union DRV830x_SPI_WRITE_WORD_REG w;
  uint16_t *cntrlReg;

  cntrlReg = (uint16_t *)&(v->Fault_Status_1);
  w.bit.R_W = READ;
  w.bit.ADDRESS = address;
  w.bit.DATA = 0;

  // Enable CS; SPI transfer; Disable CS
  digitalWrite(DRV_CS, LOW);

  cntrlReg[address] = SPI_Driver(v, w.all);

  digitalWrite(DRV_CS, HIGH);
}
void DRV8323_SPI_Write(DRV8323_VARS_t *v, uint16_t address)
{
  union DRV830x_SPI_WRITE_WORD_REG w;
  uint16_t *cntrlReg;

  cntrlReg = (uint16_t *)&(v->Fault_Status_1);
  w.bit.R_W = WRITE;
  w.bit.ADDRESS = address;
  w.bit.DATA = cntrlReg[address];

  digitalWrite(DRV_CS, LOW);
  SPI_Driver(v, w.all);
  digitalWrite(DRV_CS, HIGH);
}
void printDrv8323Regs()
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

void doDriver(char *cmd)
{
  if (cmd[0] == 'E')
  {
    if (cmd[1] == '1')
    {
      printf("Enabling driver\r\n");
      DRV_ENABLE_ON
      // driverBase.enable();
    }
    else if (cmd[1] == '0')
    {
      printf("Disabling driver\r\n");
      DRV_ENABLE_OFF
      // driverBase.disable();
    }
  }
  else if (cmd[0] == 'R')
  {
    if (cmd[1] == '1')
    {
      // driverBase.enable();
      drvReadAllRegs();
      gDrv8323.Driver_Control.bit.CLR_FLT = drv_clrFaults;
      DRV8323_SPI_Write(&gDrv8323, DRV8323_DRIVER_CNTRL_ADDR);
      printf("Cleared faults: FAULT_PIN:%d\r\n", digitalRead(DRV_NFAULT));
      // driverBase.disable();
    }
  }
  
}

void doRegisters(char *cmd)
{
  uint8_t reg_addr = (uint8_t)cmd[1] - '0'; // Convert ASCII character to integer (assuming cmd[1] is '0' to '9')
  if (cmd[0] == 'R')
  {
    if (cmd[1] == 'A')
    {
      DRV_ENABLE_ON
      drvReadAllRegs();
      printDrv8323Regs();
    }

    else if (cmd[1] != 'A')
    {
      printf("read register %d\r\n", reg_addr);
      DRV8323_SPI_Read(&gDrv8323, reg_addr);
      if ((uint8_t)reg_addr == DRV8323_FAULT_STATUS_1_ADDR)
      {
        printf("Fault Status 1 register value: %04X\r\n", gDrv8323.Fault_Status_1.all);
      }
      else if ((uint8_t)reg_addr == DRV8323_CSA_CNTRL_ADDR)
      {
        printf("CSA Control register value: %04X\r\n", gDrv8323.CSA_Control.all);
      }
      else if ((uint8_t)reg_addr == DRV8323_VGS_STATUS_2_ADDR)
      {
        printf("VGS Status 2 register value: %04X\r\n", gDrv8323.VGS_Status_2.all);
      }
      else if ((uint8_t)reg_addr == DRV8323_DRIVER_CNTRL_ADDR)
      {
        printf("Driver Control register value: %04X\r\n", gDrv8323.Driver_Control.all);
      }
      else if ((uint8_t)reg_addr == DRV8323_GATE_DRIVE_HS_ADDR)
      {
        printf("Gate Drive HS register value: %04X\r\n", gDrv8323.Gate_Drive_HS.all);
      }
      else if ((uint8_t)reg_addr == DRV8323_GATE_DRIVE_LS_ADDR)
      {
        printf("Gate Drive LS register value: %04X\r\n", gDrv8323.Gate_Drive_LS.all);
      }
      else if ((uint8_t)reg_addr == DRV8323_OCP_CNTRL_ADDR)
      {
        printf("OCP Control register value: %04X\r\n", gDrv8323.OCP_Control.all);
      }
      else if ((uint8_t)reg_addr == DRV8323_CSA_CNTRL_ADDR)
      {
        printf("CSA Control register value: %04X\r\n", gDrv8323.CSA_Control.all);
      }
    }
  }
  else if (cmd[0] == '1')
  {
    Serial3.println("change gain to 40");
    gDrv8323.CSA_Control.bit.CSA_GAIN = drv_gain_40;
    DRV8323_SPI_Write(&gDrv8323, DRV8323_CSA_CNTRL_ADDR);
  }
  else if (cmd[0] == '2')
  {
    DRV8323_SPI_Read(&gDrv8323, DRV8323_CSA_CNTRL_ADDR);
    Serial3.println("change gain to 20");
    gDrv8323.CSA_Control.bit.CSA_GAIN = drv_gain_20;
    DRV8323_SPI_Write(&gDrv8323, DRV8323_CSA_CNTRL_ADDR);
  }
}

void GPIO_INIT()
{
  pinMode(RS485_DIR_PIN, OUTPUT);
  #if defined(PCB_5410)
  pinMode(LED1, OUTPUT);  digitalWrite(LED1, HIGH);
  pinMode(LED2, OUTPUT);  digitalWrite(LED2, HIGH);
  pinMode(LED3, OUTPUT);  digitalWrite(LED3, HIGH);
  pinMode(LED4, OUTPUT);  digitalWrite(LED4, HIGH);
  #endif


  pinMode(DRV_NFAULT, INPUT);
  attachInterrupt(digitalPinToInterrupt(DRV_NFAULT), fault_ISR, CHANGE);
  pinMode(DRV_ENABLE, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(DRV_ENABLE), DRV_enable_ISR, CHANGE);

  pinMode(DRV_CAL_AMP, OUTPUT);
  pinMode(DRV_CS, OUTPUT);
  
}

void drvReadAllRegs()
{
  for (uint16_t i = 0; i < DRV8323_CSA_CNTRL_ADDR + 1; i++)
  {
    gDrv8323.CSA_Control.all = 0;
    DRV8323_SPI_Read(&gDrv8323, i);
    HAL_Delay(5);
  }
}

void setup_DRV_registers (uint8_t mode)
{
  if (mode == 0)
  {
    //________________________________________________________________DRV INIT
  drvReadAllRegs();

  gDrv8323.CSA_Control.bit.CSA_GAIN = drv_gain_40;
  gDrv8323.CSA_Control.bit.VREF_DIV = drv_vref_div_2;
  DRV8323_SPI_Write(&gDrv8323, DRV8323_CSA_CNTRL_ADDR);

  gDrv8323.Driver_Control.bit.PWM_MODE = drv_PWM_mode_6;
  DRV8323_SPI_Write(&gDrv8323, DRV8323_DRIVER_CNTRL_ADDR);

  gDrv8323.Gate_Drive_HS.bit.IDRIVEP_HS = drv_idriveP_hs_120mA;
  gDrv8323.Gate_Drive_HS.bit.IDRIVEN_HS = drv_idriveN_hs_240mA;

  gDrv8323.Gate_Drive_HS.bit._LOCK = drv_unlock;
  
  gDrv8323.Gate_Drive_LS.bit.IDRIVEP_LS = drv_idriveP_ls_120mA;
  gDrv8323.Gate_Drive_LS.bit.IDRIVEN_LS = drv_idriveN_ls_240mA;
  gDrv8323.Gate_Drive_LS.bit.TDRIVE = drv_tdrive_1000nS;

  DRV8323_SPI_Write(&gDrv8323, DRV8323_GATE_DRIVE_HS_ADDR);
  DRV8323_SPI_Write(&gDrv8323, DRV8323_GATE_DRIVE_LS_ADDR);

  // 6. Настройка Dead Time и OCP // Устанавливаем Dead Time 200ns
  gDrv8323.OCP_Control.bit.DEAD_TIME = drv_deadTime_100nS;
  // Грубая защита VDS 0.26V
  gDrv8323.OCP_Control.bit.VDS_LVL = drv_vds_lvl_260mV;
  DRV8323_SPI_Write(&gDrv8323, DRV8323_OCP_CNTRL_ADDR);
  //________________________________________________________________DRV INIT END
  }
  else if (mode == 1)
  {
    drvReadAllRegs();

    // 1. Измерение тока
    gDrv8323.CSA_Control.bit.CSA_GAIN = drv_gain_40;
    // gDrv8323.CSA_Control.bit.VREF_DIV = drv_vref_div_2;
    DRV8323_SPI_Write(&gDrv8323, DRV8323_CSA_CNTRL_ADDR);

    // 2. Режим управления
    gDrv8323.Driver_Control.bit.PWM_MODE = drv_PWM_mode_6;
    gDrv8323.Driver_Control.bit.CLR_FLT = drv_clrFaults;
    DRV8323_SPI_Write(&gDrv8323, DRV8323_DRIVER_CNTRL_ADDR);

    // 3. Накачка затворов (HS). Ток открытия ~1A, ток закрытия ~2A
    gDrv8323.Gate_Drive_HS.bit.IDRIVEP_HS = drv_idriveP_hs_140mA;
    gDrv8323.Gate_Drive_HS.bit.IDRIVEN_HS = drv_idriveN_hs_2000mA;
    gDrv8323.Gate_Drive_HS.bit._LOCK = drv_unlock;
    DRV8323_SPI_Write(&gDrv8323, DRV8323_GATE_DRIVE_HS_ADDR);
    
    // 4. Накачка затворов (LS) и время TDRIVE
    gDrv8323.Gate_Drive_LS.bit.IDRIVEP_LS = drv_idriveP_ls_120mA;
    gDrv8323.Gate_Drive_LS.bit.IDRIVEN_LS = drv_idriveN_ls_2000mA;
    // TDRIVE = 2000ns. Даем больше времени тяжелому затвору на зарядку, чтобы избежать ошибки GDF
    gDrv8323.Gate_Drive_LS.bit.TDRIVE = drv_tdrive_4000nS;
    DRV8323_SPI_Write(&gDrv8323, DRV8323_GATE_DRIVE_LS_ADDR);

    // 5. Защита и тайминги
    // Тяжелые ключи медленно закрываются. Увеличиваем Dead Time до 400ns во избежание сквозных токов
    gDrv8323.OCP_Control.bit.DEAD_TIME = drv_deadTime_400nS;
    
    // Защита VDS. 0.26V / 0.004 Ом (Wayon Rds) = лимит ~65 Ампер. Этого хватит с запасом.
    gDrv8323.OCP_Control.bit.VDS_LVL = drv_vds_lvl_1880mV;
    gDrv8323.OCP_Control.bit.OCP_DEG = drv_ocp_deg_8us;
    
    DRV8323_SPI_Write(&gDrv8323, DRV8323_OCP_CNTRL_ADDR);
  }
 
}

void fault_ISR()
{
  if (digitalRead(DRV_NFAULT))
  {
    LED4_OFF
  } else
  {
    LED4_ON
    checkFault();
  }  
}

void DRV_enable_ISR()
{
  if (digitalRead(DRV_ENABLE))
  {
    LED3_ON
  } else
  {
    LED3_OFF
  }  
}