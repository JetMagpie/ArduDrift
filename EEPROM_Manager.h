#ifndef __EEPROM_MANAGER_H__
#define __EEPROM_MANAGER_H__

#include <avr/io.h>
#include <avr/eeprom.h>
#include <stdint.h>
#include <stdbool.h>

class EEPROM_Manager {
public:
    // 构造函数，可以指定基地址
    EEPROM_Manager(uint16_t base_addr = 0);
    
    // 初始化EEPROM管理器
    void init();
    
    // 基本数据类型读写
    bool writeInt(uint16_t addr, int32_t value);
    int32_t readInt(uint16_t addr);
    
    bool writeFloat(uint16_t addr, float value);
    float readFloat(uint16_t addr);
    
    bool writeByte(uint16_t addr, uint8_t value);
    uint8_t readByte(uint16_t addr);
    
    bool writeBool(uint16_t addr, bool value);
    bool readBool(uint16_t addr);
    
    // 块读写操作
    bool writeBlock(uint16_t addr, const void* data, uint16_t size);
    bool readBlock(uint16_t addr, void* data, uint16_t size);
    
    // 字符串读写（带长度检查）
    bool writeString(uint16_t addr, const char* str, uint16_t max_len);
    bool readString(uint16_t addr, char* buffer, uint16_t max_len);
    
    // 验证EEPROM操作
    bool verifyInt(uint16_t addr, int32_t expected_value);
    bool verifyFloat(uint16_t addr, float expected_value, float tolerance = 0.0001);
    bool verifyBlock(uint16_t addr, const void* data, uint16_t size);
    
    // 获取EEPROM信息
    uint16_t getSize();
    uint16_t getBaseAddress();
    
    // 擦除操作（慎用）
    void erase(uint16_t addr, uint16_t size);
    void eraseAll();

private:
    uint16_t _base_addr;
    bool _initialized;
    
    // 地址检查
    bool _checkAddress(uint16_t addr, uint16_t size);
    
    // 智能写入（只在值改变时写入）
    bool _smartWriteByte(uint16_t addr, uint8_t value);
    
    // 浮点数比较（考虑精度误差）
    bool _compareFloat(float a, float b, float tolerance);
};

#endif // __EEPROM_MANAGER_H__