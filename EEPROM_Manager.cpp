#include "EEPROM_Manager.h"
#include <util/delay.h>
#include <string.h>
#include <math.h>

EEPROM_Manager::EEPROM_Manager(uint16_t base_addr) 
    : _base_addr(base_addr), _initialized(false) {
}

void EEPROM_Manager::init() {
    _initialized = true;
}

bool EEPROM_Manager::_checkAddress(uint16_t addr, uint16_t size) {
    uint16_t actual_addr = _base_addr + addr;
    return (actual_addr + size) <= E2END + 1;
}

bool EEPROM_Manager::_smartWriteByte(uint16_t addr, uint8_t value) {
    uint16_t actual_addr = _base_addr + addr;
    
    // 读取当前值，只有不同时才写入（延长EEPROM寿命）
    uint8_t current_value = eeprom_read_byte((const uint8_t*)actual_addr);
    if (current_value != value) {
        eeprom_write_byte((uint8_t*)actual_addr, value);
        _delay_ms(1);
        return true;
    }
    return false;
}

bool EEPROM_Manager::_compareFloat(float a, float b, float tolerance) {
    return fabs(a - b) < tolerance;
}

bool EEPROM_Manager::writeInt(uint16_t addr, int32_t value) {
    if (!_checkAddress(addr, sizeof(value))) return false;
    
    uint16_t actual_addr = _base_addr + addr;
    eeprom_write_block(&value, (void*)actual_addr, sizeof(value));
    _delay_ms(2);
    return true;
}

int32_t EEPROM_Manager::readInt(uint16_t addr) {
    int32_t value = 0;
    if (!_checkAddress(addr, sizeof(value))) return 0;
    
    uint16_t actual_addr = _base_addr + addr;
    eeprom_read_block(&value, (const void*)actual_addr, sizeof(value));
    return value;
}

bool EEPROM_Manager::writeFloat(uint16_t addr, float value) {
    if (!_checkAddress(addr, sizeof(value))) return false;
    
    uint16_t actual_addr = _base_addr + addr;
    eeprom_write_block(&value, (void*)actual_addr, sizeof(value));
    _delay_ms(2);
    return true;
}

float EEPROM_Manager::readFloat(uint16_t addr) {
    float value = 0;
    if (!_checkAddress(addr, sizeof(value))) return 0;
    
    uint16_t actual_addr = _base_addr + addr;
    eeprom_read_block(&value, (const void*)actual_addr, sizeof(value));
    return value;
}

bool EEPROM_Manager::writeByte(uint16_t addr, uint8_t value) {
    if (!_checkAddress(addr, sizeof(value))) return false;
    
    return _smartWriteByte(addr, value);
}

uint8_t EEPROM_Manager::readByte(uint16_t addr) {
    if (!_checkAddress(addr, sizeof(uint8_t))) return 0;
    
    uint16_t actual_addr = _base_addr + addr;
    return eeprom_read_byte((const uint8_t*)actual_addr);
}

bool EEPROM_Manager::writeBool(uint16_t addr, bool value) {
    return writeByte(addr, value ? 1 : 0);
}

bool EEPROM_Manager::readBool(uint16_t addr) {
    return readByte(addr) != 0;
}

bool EEPROM_Manager::writeBlock(uint16_t addr, const void* data, uint16_t size) {
    if (!_checkAddress(addr, size)) return false;
    
    uint16_t actual_addr = _base_addr + addr;
    const uint8_t* byte_data = (const uint8_t*)data;
    
    for (uint16_t i = 0; i < size; i++) {
        _smartWriteByte(addr + i, byte_data[i]);
    }
    
    return true;
}

bool EEPROM_Manager::readBlock(uint16_t addr, void* data, uint16_t size) {
    if (!_checkAddress(addr, size)) return false;
    
    uint16_t actual_addr = _base_addr + addr;
    eeprom_read_block(data, (const void*)actual_addr, size);
    return true;
}

bool EEPROM_Manager::writeString(uint16_t addr, const char* str, uint16_t max_len) {
    if (!_checkAddress(addr, max_len)) return false;
    
    uint16_t len = 0;
    
    // 计算字符串长度
    while (str[len] != '\0' && len < max_len - 1) {
        len++;
    }
    
    // 写入长度字节
    _smartWriteByte(addr, len);
    
    // 写入字符串内容
    for (uint16_t i = 0; i < len; i++) {
        _smartWriteByte(addr + 1 + i, str[i]);
    }
    
    // 确保以null结尾
    _smartWriteByte(addr + 1 + len, '\0');
    
    return true;
}

bool EEPROM_Manager::readString(uint16_t addr, char* buffer, uint16_t max_len) {
    if (!_checkAddress(addr, max_len)) return false;
    
    uint16_t actual_addr = _base_addr + addr;
    
    // 读取长度字节
    uint8_t len = eeprom_read_byte((const uint8_t*)actual_addr);
    if (len >= max_len - 1) {
        len = max_len - 2;
    }
    
    // 读取字符串内容
    for (uint16_t i = 0; i < len; i++) {
        buffer[i] = eeprom_read_byte((const uint8_t*)(actual_addr + 1 + i));
    }
    
    buffer[len] = '\0';
    return true;
}

bool EEPROM_Manager::verifyInt(uint16_t addr, int32_t expected_value) {
    int32_t read_value = readInt(addr);
    return read_value == expected_value;
}

bool EEPROM_Manager::verifyFloat(uint16_t addr, float expected_value, float tolerance) {
    float read_value = readFloat(addr);
    return _compareFloat(read_value, expected_value, tolerance);
}

bool EEPROM_Manager::verifyBlock(uint16_t addr, const void* data, uint16_t size) {
    if (!_checkAddress(addr, size)) return false;
    
    // 避免使用动态内存，改为逐个字节比较
    const uint8_t* original_data = (const uint8_t*)data;
    uint16_t actual_addr = _base_addr + addr;
    
    for (uint16_t i = 0; i < size; i++) {
        uint8_t eeprom_byte = eeprom_read_byte((const uint8_t*)(actual_addr + i));
        if (eeprom_byte != original_data[i]) {
            return false;
        }
    }
    return true;
}

uint16_t EEPROM_Manager::getSize() {
    return E2END + 1;
}

uint16_t EEPROM_Manager::getBaseAddress() {
    return _base_addr;
}

void EEPROM_Manager::erase(uint16_t addr, uint16_t size) {
    if (!_checkAddress(addr, size)) return;
    
    uint16_t actual_addr = _base_addr + addr;
    for (uint16_t i = 0; i < size; i++) {
        eeprom_write_byte((uint8_t*)(actual_addr + i), 0xFF);
    }
    _delay_ms(10);
}

void EEPROM_Manager::eraseAll() {
    erase(0, getSize());
}