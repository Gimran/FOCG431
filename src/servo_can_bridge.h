#pragma once
#include <SimpleFOC.h>
#include "comms/can/CANCommander.h"

// Функция инициализации моста (загрузка EEPROM и привязка коллбека)
void initServoCANBridge(CANCommander& commander);