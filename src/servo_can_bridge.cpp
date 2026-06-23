#include "servo_can_bridge.h"
#include "motor_settings.h"
#include <EEPROM.h>

// Актуальные адреса кастомных регистров (начиная с 0xE0)
#define REG_GEAR_RATIO   0xE0
#define REG_SERVO_TARGET 0xE1
#define REG_SERVO_ANGLE  0xE2

// Глобальная переменная для редукции
float gear_ratio = GEAR_RATIO; 
const int EEPROM_ADDR_GEAR = 0; 

// --- Логика работы с памятью ---
void loadSettings() {
    float eeprom_val = 0.0f;
    EEPROM.get(EEPROM_ADDR_GEAR, eeprom_val);
    
    if (isnan(eeprom_val) || eeprom_val < 0.001f || eeprom_val > 10000.0f) {
        // Оставляем дефолтный gear_ratio
    } else {
        gear_ratio = eeprom_val;
    }
}

void saveSettings() {
    EEPROM.put(EEPROM_ADDR_GEAR, gear_ratio);
}


// ================= ОБРАБОТЧИКИ РЕГИСТРОВ =================

// --- Регистр 0xE0: Передаточное число (Gear Ratio) ---
bool readGearRatio(RegisterIO& io, FOCMotor* m) { 
    io << gear_ratio; 
    return true; 
}
bool writeGearRatio(RegisterIO& io, FOCMotor* m) { 
    io >> gear_ratio; 
    saveSettings(); 
    return true; 
}

// --- Регистр 0xE1: Целевой угол (Target) ---
bool readServoTarget(RegisterIO& io, FOCMotor* m) { 
    // Возвращаем целевой угол шарнира (делим таргет мотора на редукцию)
    io << (float)(m->target / gear_ratio); 
    return true; 
}
bool writeServoTarget(RegisterIO& io, FOCMotor* m) { 
    float target_servo_angle; 
    io >> target_servo_angle; 
    m->target = target_servo_angle * gear_ratio; 
    return true; 
}

// --- Регистр 0xE2: Текущий угол (Angle) ---
bool readServoAngle(RegisterIO& io, FOCMotor* m) { 
    // Возвращаем реальный угол шарнира
    io << (float)(m->shaft_angle / gear_ratio); 
    return true; 
}
bool writeServoAngle(RegisterIO& io, FOCMotor* m) { 
    // Только для чтения, не даем ничего записать
    return false; 
}


// --- Инициализация модуля ---
void initServoCANBridge(CANCommander& commander) {
    loadSettings();
    
    // Регистрируем кастомные регистры: (Адрес, Размер в байтах (float = 4), Функция Чтения, Функция Записи)
    commander.addCustomRegister(REG_GEAR_RATIO, 4, readGearRatio, writeGearRatio);
    commander.addCustomRegister(REG_SERVO_TARGET, 4, readServoTarget, writeServoTarget);
    commander.addCustomRegister(REG_SERVO_ANGLE, 4, readServoAngle, writeServoAngle);
}