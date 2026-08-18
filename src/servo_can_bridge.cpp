#include "servo_can_bridge.h"
#include "motor_settings.h"
#include <EEPROM.h>
#include <LibPrintf.h>
#include "mbs_encoder.h"

// Актуальные адреса кастомных регистров (0xE0 .. 0xEB)
#define REG_GEAR_RATIO   0xE0 // [R/W] Передаточное число
#define REG_SERVO_TARGET 0xE1 // [W]   Целевой угол шарнира (рад)
#define REG_SERVO_ANGLE  0xE2 // [R]   Текущий угол шарнира (рад)
#define REG_SERVO_VEL    0xE3 // [R]   Текущая скорость шарнира (рад/с)
#define REG_VOLTAGE      0xE4 // [R]   Напряжение питания (Вольт)
#define REG_TEMPERATURE  0xE5 // [R]   Температура мотора (°C)
#define REG_CANID        0xE6 // [R/W] CANID (1..254, применяется после перезагрузки)
#define REG_REBOOT       0xE7 // [W]   Запись 1 - штатное отключение двигателя и перезагрузка MCU
#define REG_ENC_ANGLE    0xE8 // [R]   Угол с энкодера MBS (рад, singleturn)
#define REG_ENC_RPM      0xE9 // [R]   Скорость с энкодера MBS (об/с)
#define REG_ENC_TEMP     0xEA // [R]   Температура кристалла энкодера MBS (°C)
#define REG_ENC_STATUS   0xEB // [R]   Статус-байт энкодера MBS (см. ENC_Status_t)
#define REG_BOOTLOADER   0xEC // [R/W] Чтение: 1 - Katapult найден во flash. Запись 1 - переход в бутлоадер Katapult

// Подтягиваем функции из main.cpp
extern float getVoltage();
extern float getTemperature();
extern ServoCANCommander commander;

float gear_ratio = GEAR_RATIO;
const int EEPROM_ADDR_GEAR = 0;
const int EEPROM_ADDR_CANID = 4; // gear_ratio занимает байты 0..3

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

// Читаем сохранённый CANID из EEPROM (до commander.init(), чтобы аппаратный фильтр настроился на него)
uint8_t loadCANID(uint8_t default_id) {
    uint8_t stored_id = 0xFF;
    EEPROM.get(EEPROM_ADDR_CANID, stored_id);
    if (stored_id >= 1 && stored_id <= 254) { // 0xFF - broadcast/чистая память, 0 - зарезервирован
        return stored_id;
    }
    return default_id;
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

// Штатное отключение двигателя и программный сброс MCU
void systemReboot(FOCMotor* m) {
    if (m) m->disable(); // PWM в 0 и выключение драйвера
    printf("Rebooting...\r\n");
    _delay(50); // даём UART допечатать
    NVIC_SystemReset();
}

// --- 0xE6: CANID ---
bool readCANID(RegisterIO& io, FOCMotor* m) {
    io << (uint8_t)commander.address;
    return true;
}
bool writeCANID(RegisterIO& io, FOCMotor* m) {
    uint8_t new_id = 0;
    io >> new_id;
    if (new_id < 1 || new_id > 254) return false; // 0 и 0xFF (broadcast) запрещены
    EEPROM.put(EEPROM_ADDR_CANID, new_id);
    // Аппаратный фильтр CAN настроен на старый ID, поэтому новый вступит в силу после перезагрузки
    printf("CANID %d saved, reboot to apply\r\n", new_id);
    return true;
}

// --- 0xE8..0xEB: Энкодер MBS (каждое чтение - свежий запрос по RS485) ---
// При сбое линка (нет ответа/CRC) отдаём NaN вместо кэша, чтобы по CAN сбой
// был отличим от честного нуля; причина печатается в UART.
static float encValueOrNaN(float v) {
    if (mbs_link_err != MBS_ERR_NONE) {
        printf("MBS link error: %s\r\n", mbs_link_err == MBS_ERR_TIMEOUT ? "timeout (no response)" : "CRC");
        return NAN;
    }
    return v;
}
bool readEncAngle(RegisterIO& io, FOCMotor* m)  { io << encValueOrNaN(readMySensorCallback()); return true; }
bool readEncRPM(RegisterIO& io, FOCMotor* m)    { io << encValueOrNaN(Get_RPM());              return true; }
bool readEncTemp(RegisterIO& io, FOCMotor* m)   { io << encValueOrNaN(MBS_GetTemperature());   return true; }
bool readEncStatus(RegisterIO& io, FOCMotor* m) {
    uint8_t s = MBS_GetStatus();
    if (mbs_link_err != MBS_ERR_NONE) s = 0xFF; // маркер "нет связи с энкодером"
    io << s;
    return true;
}
bool writeEncReg(RegisterIO& io, FOCMotor* m)   { return false; } // read-only

// --- 0xEC: Katapult bootloader ---
// Вход в бутлоадер по механизму Katapult/Klipper (см. klipper src/generic/armcm_reset.c):
// приложение пишет 64-битный код запроса по адресу вершины стека бутлоадера
// (слово 0 его таблицы векторов по 0x08000000) и делает сброс MCU.
#define KATAPULT_SIGNATURE 0x21746f6f426e6143ULL // "CanBoot!"
#define KATAPULT_REQUEST   0x5984E3FA6CA1589BULL

// Возвращает адрес ячейки запроса, если Katapult прошит с 0x08000000, иначе nullptr
static uint64_t* katapultRequestCell() {
    uint32_t* bl_vectors = (uint32_t*)FLASH_BASE;
    uint32_t boot_sig_addr = bl_vectors[1] - 9; // сигнатура лежит перед Reset_Handler бутлоадера
    uint32_t req_sig_addr  = bl_vectors[0];     // начальный SP бутлоадера = ячейка запроса (в RAM)
    if ((boot_sig_addr & 7) || (req_sig_addr & 7)) return nullptr;
    if (boot_sig_addr < FLASH_BASE || boot_sig_addr >= FLASH_BASE + 0x2000) return nullptr;   // внутри 8К бутлоадера
    if (req_sig_addr < SRAM_BASE || req_sig_addr > SRAM_BASE + 0x8000 - 8) return nullptr;    // внутри 32К RAM
    if (*(uint64_t*)boot_sig_addr != KATAPULT_SIGNATURE) return nullptr;
    return (uint64_t*)req_sig_addr;
}

// Переход в Katapult: запись кода запроса и сброс MCU. false - бутлоадер не найден
static bool enterKatapultBootloader(FOCMotor* m) {
    uint64_t* req_sig = katapultRequestCell();
    if (!req_sig) {
        printf("Katapult not found at 0x08000000\r\n");
        return false;
    }
    if (m) m->disable();
    printf("Entering Katapult bootloader...\r\n");
    _delay(50); // даём UART допечатать
    __disable_irq();
    *req_sig = KATAPULT_REQUEST;
    NVIC_SystemReset();
    return true; // не достигается
}

bool readBootloader(RegisterIO& io, FOCMotor* m) {
    io << (uint8_t)(katapultRequestCell() ? 1 : 0);
    return true;
}
bool writeBootloader(RegisterIO& io, FOCMotor* m) {
    uint8_t val = 0;
    io >> val;
    if (val != 1) return false; // защита от случайной записи
    return enterKatapultBootloader(m);
}

// --- 0xE7: Reboot ---
bool readReboot(RegisterIO& io, FOCMotor* m) { io << (uint8_t)0; return true; }
bool writeReboot(RegisterIO& io, FOCMotor* m) {
    uint8_t val = 0;
    io >> val;
    if (val != 1) return false; // защита от случайной записи
    systemReboot(m);
    return true; // не достигается
}


// --- Инициализация ---
void initServoCANBridge(CANCommander& commander) {
    loadSettings();
    
    commander.addCustomRegister(REG_GEAR_RATIO,   4, readGearRatio,   writeGearRatio);
    commander.addCustomRegister(REG_SERVO_TARGET, 4, readServoTarget, writeServoTarget);
    commander.addCustomRegister(REG_SERVO_ANGLE,  4, readServoAngle,  writeServoAngle);
    commander.addCustomRegister(REG_SERVO_VEL,    4, readServoVel,    writeServoVel);
    commander.addCustomRegister(REG_VOLTAGE,      4, readVoltage,     writeVoltage);
    commander.addCustomRegister(REG_TEMPERATURE,  4, readTemperature, writeTemperature);
    commander.addCustomRegister(REG_CANID,        1, readCANID,       writeCANID);
    commander.addCustomRegister(REG_REBOOT,       1, readReboot,      writeReboot);
    commander.addCustomRegister(REG_ENC_ANGLE,    4, readEncAngle,    writeEncReg);
    commander.addCustomRegister(REG_ENC_RPM,      4, readEncRPM,      writeEncReg);
    commander.addCustomRegister(REG_ENC_TEMP,     4, readEncTemp,     writeEncReg);
    commander.addCustomRegister(REG_ENC_STATUS,   1, readEncStatus,   writeEncReg);
    commander.addCustomRegister(REG_BOOTLOADER,   1, readBootloader,  writeBootloader);
}

// ================= Admin-протокол Katapult/Klipper (CAN ID 0x3f0) =================
// Штатный katapult flashtool.py общается с приложением стандартными (11-бит) кадрами:
//   query (-q):  0x3f0 [0x00] -> ответ 0x3f1 [0x20, UUID x6, 0x01]
//   reboot (-r): 0x3f0 [0x02, UUID x6] -> переход в бутлоадер
// UUID узла = младшие 6 байт fasthash64 от 96-битного UID чипа - так же его
// вычисляют Katapult и Klipper, поэтому UUID совпадает с показанным бутлоадером.

#define CANBUS_ID_ADMIN             0x3f0
#define CANBUS_ID_ADMIN_RESP        0x3f1
#define CANBUS_CMD_QUERY_UNASSIGNED 0x00
#define KLIPPER_CMD_SET_NODEID      0x01 // в ответе на query: признак "приложение", не бутлоадер
#define KLIPPER_CMD_REQ_BOOTLOADER  0x02
#define CANBUS_RESP_NEED_NODEID     0x20

static uint8_t can_uuid[6];

// fasthash64 (MIT, Zilong Tan) - хеш, которым Katapult/Klipper считают UUID из UID чипа
static inline uint64_t fh_mix(uint64_t h) {
    h ^= h >> 23;
    h *= 0x2127599bf4325c37ULL;
    h ^= h >> 47;
    return h;
}
static uint64_t fasthash64(const uint8_t* buf, size_t len, uint64_t seed) {
    const uint64_t m = 0x880355f21e6d1965ULL;
    uint64_t h = seed ^ (len * m);
    size_t n8 = len / 8;
    for (size_t i = 0; i < n8; i++) {
        uint64_t v;
        memcpy(&v, buf + i * 8, 8);
        h ^= fh_mix(v);
        h *= m;
    }
    size_t tail = len & 7;
    if (tail) {
        uint64_t v = 0;
        for (size_t i = 0; i < tail; i++) v |= (uint64_t)buf[n8 * 8 + i] << (8 * i);
        h ^= fh_mix(v);
        h *= m;
    }
    return fh_mix(h);
}

// UUID считается один раз: он нужен и admin-протоколу, и дефолтному CANID,
// поэтому адрес узла предсказуем по тому же UUID, что показывает flashtool.
static bool uuid_ready = false;
static void ensureUUID() {
    if (uuid_ready) return;
    uint64_t hash = fasthash64((const uint8_t*)UID_BASE, 12, 0xA16231A7ULL);
    memcpy(can_uuid, &hash, sizeof(can_uuid));
    uuid_ready = true;
}

// Дефолтный адрес узла из UUID: младшие разряды укладываем в 1..254.
// Нужен, чтобы несколько плат на одной линии не стартовали с одним адресом.
// Явно заданный регистром 0xE6 адрес хранится в EEPROM и имеет приоритет.
uint8_t defaultCANID() {
    ensureUUID();
    uint32_t v = ((uint32_t)can_uuid[0] << 16) | ((uint32_t)can_uuid[1] << 8) | can_uuid[2];
    return (uint8_t)(1 + (v % 254));   // 0 и 255 зарезервированы
}

void ServoCANCommander::init() {
    CANCommander::init();

    ensureUUID();
    printf("Katapult UUID: %02x%02x%02x%02x%02x%02x\r\n",
           can_uuid[0], can_uuid[1], can_uuid[2],
           can_uuid[3], can_uuid[4], can_uuid[5]);

    // Базовый аппаратный фильтр пропускает только extended-кадры нашего адреса.
    // Дополнительно принимаем стандартный ID 0x3f0 (admin-запросы flashtool).
    FDCAN_FilterTypeDef f = {};
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = 0;
    f.FilterType = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1 = CANBUS_ID_ADMIN;
    f.FilterID2 = 0x7FF;
    HAL_FDCAN_ConfigFilter(&STM_FDCAN::hcan_, &f);
}

void ServoCANCommander::handleCANMessage(CanMsg& msg) {
    if (!msg.isExtendedId() && msg.getStandardId() == CANBUS_ID_ADMIN) {
        if (msg.data_length >= 1 && msg.data[0] == CANBUS_CMD_QUERY_UNASSIGNED) {
            // flashtool -q: сообщаем UUID ("Application: Klipper" в его выводе)
            uint8_t buf[8];
            buf[0] = CANBUS_RESP_NEED_NODEID;
            memcpy(&buf[1], can_uuid, sizeof(can_uuid));
            buf[7] = KLIPPER_CMD_SET_NODEID;
            CanMsg resp(CanStandardId(CANBUS_ID_ADMIN_RESP), sizeof(buf), buf);
            _can->write(resp);
        } else if (msg.data_length >= 7 && msg.data[0] == KLIPPER_CMD_REQ_BOOTLOADER
                   && memcmp(&msg.data[1], can_uuid, sizeof(can_uuid)) == 0) {
            // flashtool -r (и автоматически перед -f): запрос перехода в бутлоадер
            enterKatapultBootloader(numMotors ? motors[0] : nullptr);
        }
        return; // чужие admin-кадры игнорируем
    }
    CANCommander::handleCANMessage(msg);
}