#include "servo_can_bridge.h"
#include "motor_settings.h"
#include <EEPROM.h>

#define REG_GEAR_RATIO   0xE0 
#define REG_SERVO_TARGET 0xE1 
#define REG_SERVO_ANGLE  0xE2 
#define REG_SERVO_VEL    0xE3 
#define REG_VOLTAGE      0xE4 
#define REG_TEMPERATURE  0xE5 
#define REG_CAN_ID       0xE6 

extern float getVoltage();
extern float getTemperature();

float gear_ratio = GEAR_RATIO; 
static uint32_t active_can_id = 1; // Собственное хранилище ID

const int EEPROM_ADDR_GEAR   = 0; // байты [0..3]
const int EEPROM_ADDR_CAN_ID = 4; // байты [4..7]

void loadSettings(uint32_t default_id) {
    active_can_id = default_id;

    // 1. Загрузка редуктора
    float eeprom_gear = 0.0f;
    EEPROM.get(EEPROM_ADDR_GEAR, eeprom_gear);
    if (!isnan(eeprom_gear) && eeprom_gear >= 0.001f && eeprom_gear <= 10000.0f) {
        gear_ratio = eeprom_gear;
    }

    // 2. Загрузка CAN ID
    uint32_t eeprom_id = 0;
    EEPROM.get(EEPROM_ADDR_CAN_ID, eeprom_id);
    if (eeprom_id >= 1 && eeprom_id <= 127) {
        active_can_id = eeprom_id;
    }
}

void saveSettings() {
    EEPROM.put(EEPROM_ADDR_GEAR, gear_ratio);
    EEPROM.put(EEPROM_ADDR_CAN_ID, active_can_id);
}

// ================= ОБРАБОТЧИКИ РЕГИСТРОВ =================

bool readGearRatio(RegisterIO& io, FOCMotor* m) { io << gear_ratio; return true; }
bool writeGearRatio(RegisterIO& io, FOCMotor* m) { io >> gear_ratio; saveSettings(); return true; }

bool readServoTarget(RegisterIO& io, FOCMotor* m) { io << (float)(m->target / gear_ratio); return true; }
bool writeServoTarget(RegisterIO& io, FOCMotor* m) { 
    float target_servo_angle; 
    io >> target_servo_angle; 
    m->target = target_servo_angle * gear_ratio; 
    return true; 
}

bool readServoAngle(RegisterIO& io, FOCMotor* m) { io << (float)(m->shaft_angle / gear_ratio); return true; }
bool writeServoAngle(RegisterIO& io, FOCMotor* m) { return false; }

bool readServoVel(RegisterIO& io, FOCMotor* m) { io << (float)(m->shaft_velocity / gear_ratio); return true; }
bool writeServoVel(RegisterIO& io, FOCMotor* m) { return false; }

bool readVoltage(RegisterIO& io, FOCMotor* m) { io << getVoltage(); return true; }
bool writeVoltage(RegisterIO& io, FOCMotor* m) { return false; }

bool readTemperature(RegisterIO& io, FOCMotor* m) { io << getTemperature(); return true; }
bool writeTemperature(RegisterIO& io, FOCMotor* m) { return false; }

// --- 0xE6: Чистая работа со своей переменной ---
bool readCanId(RegisterIO& io, FOCMotor* m) { 
    io << active_can_id; 
    return true; 
}

bool writeCanId(RegisterIO& io, FOCMotor* m) { 
    uint32_t new_id = 0;
    io >> new_id; 

    if (new_id >= 1 && new_id <= 127) {
        active_can_id = new_id; 
        saveSettings();         
        printf("[CAN_BRIDGE] ID %lu saved to EEPROM.\r\n", new_id);
    }
    return true; 
}

// --- Инициализация ---
void initServoCANBridge(CANCommander& commander, uint32_t default_can_id) {
    loadSettings(default_can_id); 
    
    commander.addCustomRegister(REG_GEAR_RATIO,   4, readGearRatio,   writeGearRatio);
    commander.addCustomRegister(REG_SERVO_TARGET, 4, readServoTarget, writeServoTarget);
    commander.addCustomRegister(REG_SERVO_ANGLE,  4, readServoAngle,  writeServoAngle);
    commander.addCustomRegister(REG_SERVO_VEL,    4, readServoVel,    writeServoVel);
    commander.addCustomRegister(REG_VOLTAGE,      4, readVoltage,     writeVoltage);
    commander.addCustomRegister(REG_TEMPERATURE,  4, readTemperature, writeTemperature);
    commander.addCustomRegister(REG_CAN_ID,       4, readCanId,       writeCanId);
}