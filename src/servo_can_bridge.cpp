#include "servo_can_bridge.h"
#include "motor_settings.h"
#include <EEPROM.h>

// Актуальные адреса кастомных регистров (0xE0 .. 0xE5)
#define REG_GEAR_RATIO   0xE0 // [R/W] Передаточное число
#define REG_SERVO_TARGET 0xE1 // [W]   Целевой угол шарнира (рад)
#define REG_SERVO_ANGLE  0xE2 // [R]   Текущий угол шарнира (рад)
#define REG_SERVO_VEL    0xE3 // [R]   Текущая скорость шарнира (рад/с)
#define REG_VOLTAGE      0xE4 // [R]   Напряжение питания (Вольт)
#define REG_TEMPERATURE  0xE5 // [R]   Температура мотора (°C)

// Подтягиваем функции из main.cpp
extern float getVoltage();
extern float getTemperature();

float gear_ratio = GEAR_RATIO; 
const int EEPROM_ADDR_GEAR = 0; 

void loadSettings() {
    float eeprom_val = 0.0f;
    EEPROM.get(EEPROM_ADDR_GEAR, eeprom_val);
    if (!isnan(eeprom_val) && eeprom_val >= 0.001f && eeprom_val <= 10000.0f) {
        gear_ratio = eeprom_val;
    }
}

void saveSettings() {
    EEPROM.put(EEPROM_ADDR_GEAR, gear_ratio);
}

// ================= ОБРАБОТЧИКИ РЕГИСТРОВ =================

// --- 0xE0: Gear Ratio ---
bool readGearRatio(RegisterIO& io, FOCMotor* m) { io << gear_ratio; return true; }
bool writeGearRatio(RegisterIO& io, FOCMotor* m) { io >> gear_ratio; saveSettings(); return true; }

// --- 0xE1: Target Angle ---
bool readServoTarget(RegisterIO& io, FOCMotor* m) { io << (float)(m->target / gear_ratio); return true; }
bool writeServoTarget(RegisterIO& io, FOCMotor* m) { 
    float target_servo_angle; 
    io >> target_servo_angle; 
    m->target = target_servo_angle * gear_ratio; 
    return true; 
}

// --- 0xE2: Actual Angle ---
bool readServoAngle(RegisterIO& io, FOCMotor* m) { 
    io << (float)(m->shaft_angle / gear_ratio); 
    return true; 
}
bool writeServoAngle(RegisterIO& io, FOCMotor* m) { return false; }

// --- 0xE3: Actual Velocity (НОВОЕ) ---
bool readServoVel(RegisterIO& io, FOCMotor* m) { 
    // Скорость выходного вала редуктора в рад/сек
    io << (float)(m->shaft_velocity / gear_ratio); 
    return true; 
}
bool writeServoVel(RegisterIO& io, FOCMotor* m) { return false; } // read-only

// --- 0xE4: Voltage (НОВОЕ) ---
bool readVoltage(RegisterIO& io, FOCMotor* m) { 
    io << getVoltage(); 
    return true; 
}
bool writeVoltage(RegisterIO& io, FOCMotor* m) { return false; }

// --- 0xE5: Temperature (НОВОЕ) ---
bool readTemperature(RegisterIO& io, FOCMotor* m) { 
    io << getTemperature(); 
    return true; 
}
bool writeTemperature(RegisterIO& io, FOCMotor* m) { return false; }


// --- Инициализация ---
void initServoCANBridge(CANCommander& commander) {
    loadSettings();
    
    commander.addCustomRegister(REG_GEAR_RATIO,   4, readGearRatio,   writeGearRatio);
    commander.addCustomRegister(REG_SERVO_TARGET, 4, readServoTarget, writeServoTarget);
    commander.addCustomRegister(REG_SERVO_ANGLE,  4, readServoAngle,  writeServoAngle);
    commander.addCustomRegister(REG_SERVO_VEL,    4, readServoVel,    writeServoVel);
    commander.addCustomRegister(REG_VOLTAGE,      4, readVoltage,     writeVoltage);
    commander.addCustomRegister(REG_TEMPERATURE,  4, readTemperature, writeTemperature);
}