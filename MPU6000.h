#ifndef __MPU6000_H__
#define __MPU6000_H__

#include <stdint.h>
#include <avr/io.h>

class MPU6000 {
public:
    MPU6000();
    
    bool init();
    bool read_sensors();
    
    // 获取传感器数据
    float get_accel_x() const { return _accel_x; }
    float get_accel_y() const { return _accel_y; }
    float get_accel_z() const { return _accel_z; }
    float get_gyro_x() const { return _gyro_x; }
    float get_gyro_y() const { return _gyro_y; }
    float get_gyro_z() const { return _gyro_z; }
    float get_temperature() const { return _temperature; }
    
    // 工具函数
    void delay_ms(uint16_t ms);
    
private:
    // 传感器数据
    float _accel_x, _accel_y, _accel_z;
    float _gyro_x, _gyro_y, _gyro_z;
    float _temperature;
    
    // SPI相关函数
    void spi_init();
    void spi_high_speed();  // 添加这行声明
    uint8_t spi_transfer(uint8_t data);
    void cs_low();
    void cs_high();

    void disable_other_spi_devices();// 禁用其他SPI设备
    
    // MPU6000寄存器操作
    uint8_t read_register(uint8_t reg);
    void write_register(uint8_t reg, uint8_t value);
    uint16_t read_register_16(uint8_t reg);
    
    // MPU6000寄存器地址
    static const uint8_t MPUREG_WHOAMI = 0x75;
    static const uint8_t MPUREG_PWR_MGMT_1 = 0x6B;
    static const uint8_t MPUREG_SMPLRT_DIV = 0x19;
    static const uint8_t MPUREG_CONFIG = 0x1A;
    static const uint8_t MPUREG_GYRO_CONFIG = 0x1B;
    static const uint8_t MPUREG_ACCEL_CONFIG = 0x1C;
    static const uint8_t MPUREG_ACCEL_XOUT_H = 0x3B;
    static const uint8_t MPUREG_ACCEL_XOUT_L = 0x3C;
    static const uint8_t MPUREG_ACCEL_YOUT_H = 0x3D;
    static const uint8_t MPUREG_ACCEL_YOUT_L = 0x3E;
    static const uint8_t MPUREG_ACCEL_ZOUT_H = 0x3F;
    static const uint8_t MPUREG_ACCEL_ZOUT_L = 0x40;
    static const uint8_t MPUREG_TEMP_OUT_H = 0x41;
    static const uint8_t MPUREG_TEMP_OUT_L = 0x42;
    static const uint8_t MPUREG_GYRO_XOUT_H = 0x43;
    static const uint8_t MPUREG_GYRO_XOUT_L = 0x44;
    static const uint8_t MPUREG_GYRO_YOUT_H = 0x45;
    static const uint8_t MPUREG_GYRO_YOUT_L = 0x46;
    static const uint8_t MPUREG_GYRO_ZOUT_H = 0x47;
    static const uint8_t MPUREG_GYRO_ZOUT_L = 0x48;
    
    // 引脚定义 (APM2.8)
    static const uint8_t MPU_CS_PIN = 53;    // PORTB0
    static const uint8_t MOSI_PIN = 51;      // PORTB2
    static const uint8_t MISO_PIN = 50;      // PORTB3
    static const uint8_t SCK_PIN = 52;       // PORTB1
};

#endif // __MPU6000_H__