#pragma once
#include <SimpleFOC.h>
#include "comms/can/CANCommander.h"

// CANCommander с поддержкой admin-протокола Katapult/Klipper (стандартный CAN ID 0x3f0).
// Позволяет штатному katapult flashtool.py обнаруживать устройство (-q, ответ с UUID)
// и переводить его в бутлоадер (-r, а также автоматически перед прошивкой -f).
class ServoCANCommander : public CANCommander {
public:
    using CANCommander::CANCommander;
    void init() override;
protected:
    void handleCANMessage(CanMsg& msg) override;
};

// Функция инициализации моста (загрузка EEPROM и привязка коллбека)
void initServoCANBridge(CANCommander& commander);

// Дефолтный CANID из уникального ID чипа (1..254).
// Применяется, если в EEPROM нет сохранённого адреса.
uint8_t defaultCANID();

// Читает сохранённый CANID из EEPROM, при отсутствии/некорректном значении возвращает default_id.
// Вызывать до commander.init(), чтобы аппаратный CAN-фильтр настроился на актуальный ID.
uint8_t loadCANID(uint8_t default_id);