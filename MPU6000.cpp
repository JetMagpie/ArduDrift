#include "MPU6000.h"
#include "UART.h"
#include <util/delay.h>

extern UART uart;

MPU6000::MPU6000() 
    : _accel_x(0), _accel_y(0), _accel_z(0),
      _gyro_x(0), _gyro_y(0), _gyro_z(0),
      _temperature(0)
{
}

void MPU6000::delay_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        _delay_ms(1);
    }
}

void MPU6000::disable_other_spi_devices() {
    uart.println("Disabling other SPI devices...");
    
    // 根据APM2.8原理图，禁用其他SPI设备的CS引脚
    
    // MS5611气压计 CS - PIN 40 (PORTG1)
    DDRG |= (1 << DDG1);   // 设置为输出
    PORTG |= (1 << PORTG1); // 设置为高电平（不选中）
    
    // 数据闪存 CS - PIN 28 (PORTA6)  
    DDRA |= (1 << DDA6);    // 设置为输出
    PORTA |= (1 << PORTA6); // 设置为高电平（不选中）
    
    // 光流传感器 CS - PIN A3 (PORTF3)
    DDRF |= (1 << DDF3);    // 设置为输出  
    PORTF |= (1 << PORTF3); // 设置为高电平（不选中）
    
    // 确保MPU6000 CS初始为高
    DDRB |= (1 << DDB0);    // CS引脚设为输出
    PORTB |= (1 << PORTB0); // 初始为高电平
    
    uart.println("Other SPI devices disabled");
}

void MPU6000::spi_init() {
    uart.println("Initializing SPI for MPU6000...");
    
    // 首先禁用其他SPI设备
    disable_other_spi_devices();
    
    // 设置MOSI, SCK为输出
    DDRB |= (1 << DDB2) | (1 << DDB1);
    // 设置MISO为输入
    DDRB &= ~(1 << DDB3);
    
    // 重要：先禁用SPI，再重新配置
    SPCR = 0;
    
    // 根据ArduPilot配置设置SPI
    // 低速模式: 500kHz (初始化用)
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1); // fosc/64
    SPSR = (1 << SPI2X); // 2倍速 -> 实际为fosc/32 = 500kHz @ 16MHz
    
    uart.println("SPI configured for 500kHz (init mode)");
    
    delay_ms(10);
}

// 切换到高速模式 (8MHz)
void MPU6000::spi_high_speed() {
    // 禁用SPI
    SPCR = 0;
    // 重新配置为高速模式
    SPCR = (1 << SPE) | (1 << MSTR); // fosc/4
    SPSR = (1 << SPI2X); // 2倍速 -> 实际为fosc/2 = 8MHz @ 16MHz
    uart.println("SPI switched to 8MHz (high speed mode)");
}

uint8_t MPU6000::spi_transfer(uint8_t data) {
    SPDR = data;
    
    // 添加超时保护
    uint16_t timeout = 0;
    while (!(SPSR & (1 << SPIF)) && timeout < 1000) {
        timeout++;
        _delay_us(1);
    }
    
    if (timeout >= 1000) {
        uart.println("SPI transfer timeout!");
        return 0xFF;
    }
    
    return SPDR;
}

void MPU6000::cs_low() {
    _delay_us(2);
    PORTB &= ~(1 << PORTB0);
    _delay_us(2);
}

void MPU6000::cs_high() {
    _delay_us(2);
    PORTB |= (1 << PORTB0);
    _delay_us(2);
}

uint8_t MPU6000::read_register(uint8_t reg) {
    uint8_t value;
    
    cs_low();
    spi_transfer(reg | 0x80); // 设置读位
    value = spi_transfer(0x00);
    cs_high();
    
    return value;
}

void MPU6000::write_register(uint8_t reg, uint8_t value) {
    cs_low();
    spi_transfer(reg & 0x7F); // 清除读位
    spi_transfer(value);
    cs_high();
}

uint16_t MPU6000::read_register_16(uint8_t reg) {
    uint16_t value;
    
    cs_low();
    spi_transfer(reg | 0x80);
    uint8_t high = spi_transfer(0x00);
    uint8_t low = spi_transfer(0x00);
    cs_high();
    
    value = (high << 8) | low;
    return value;
}

bool MPU6000::init() {
    uart.println("=== MPU6000 Initialization ===");
    
    // 初始化SPI（低速模式）
    spi_init();
    delay_ms(100);
    
    // 测试1: 直接读取WHO_AM_I
    uart.println("Test 1: Reading WHO_AM_I register...");
    uint8_t whoami = read_register(MPUREG_WHOAMI);
    uart.print("WHO_AM_I: 0x");
    uart.printHex(whoami);
    uart.println();
    
    if (whoami == 0xFF) {
        uart.println("ERROR: SPI communication failed (all 0xFF)");
        
        // 更详细的SPI诊断
        uart.println("Running detailed SPI diagnostics...");
        
        // 检查SPI寄存器状态
        uart.print("SPCR: 0x");
        uart.printHex(SPCR);
        uart.print(", SPSR: 0x");
        uart.printHex(SPSR);
        uart.print(", SPDR: 0x");
        uart.printHex(SPDR);
        uart.println();
        
        // 检查端口状态
        uart.print("PORTB: 0x");
        uart.printHex(PORTB);
        uart.print(", DDRB: 0x");
        uart.printHex(DDRB);
        uart.print(", PINB: 0x");
        uart.printHex(PINB);
        uart.println();
        
        // 测试SPI环回（需要连接MOSI到MISO）
        uart.println("Testing SPI loopback (connect MOSI to MISO)...");
        cs_low();
        uint8_t sent_byte = 0xAA;
        spi_transfer(sent_byte);
        uint8_t loopback = spi_transfer(0x00);
        cs_high();
        uart.print("Loopback test: Sent 0x");
        uart.printHex(sent_byte);
        uart.print(", Received 0x");
        uart.printHex(loopback);
        uart.println();
        
        if (loopback == sent_byte) {
            uart.println("SPI hardware is working!");
        } else {
            uart.println("SPI hardware test FAILED!");
        }
        
        return false;
    }
    
    if (whoami != 0x68) {
        uart.println("ERROR: Wrong device ID");
        uart.print("Expected: 0x68, Got: 0x");
        uart.printHex(whoami);
        uart.println();
        return false;
    }
    
    uart.println("MPU6000 detected successfully!");
    
    // 复位设备
    uart.println("Resetting MPU6000...");
    write_register(MPUREG_PWR_MGMT_1, 0x80);
    
    // 等待复位完成
    uint8_t timeout = 0;
    do {
        delay_ms(10);
        timeout++;
    } while (read_register(MPUREG_PWR_MGMT_1) & 0x80 && timeout < 50);
    
    if (timeout >= 50) {
        uart.println("ERROR: Reset timeout");
        return false;
    }
    
    uart.println("Reset completed");
    delay_ms(100);
    
    // 配置MPU6000
    uart.println("Configuring MPU6000...");
    
    // 唤醒设备，选择陀螺仪X轴为时钟源
    write_register(MPUREG_PWR_MGMT_1, 0x01);
    delay_ms(10);
    
    // 设置采样率分频器
    write_register(MPUREG_SMPLRT_DIV, 0x07); // 125Hz
    
    // 设置数字低通滤波器
    write_register(MPUREG_CONFIG, 0x06); // 5Hz
    
    // 设置陀螺仪量程 ±2000°/s
    write_register(MPUREG_GYRO_CONFIG, 0x18);
    
    // 设置加速度计量程 ±8g  
    write_register(MPUREG_ACCEL_CONFIG, 0x10);
    
    delay_ms(100);
    
    // 切换到高速SPI模式进行数据读取
    spi_high_speed();
    
    // 验证配置
    uart.println("Verifying configuration...");
    uint8_t gyro_cfg = read_register(MPUREG_GYRO_CONFIG);
    uint8_t accel_cfg = read_register(MPUREG_ACCEL_CONFIG);
    
    uart.print("Gyro config: 0x");
    uart.printHex(gyro_cfg);
    uart.println();
    
    uart.print("Accel config: 0x");
    uart.printHex(accel_cfg);
    uart.println();
    
    // 测试读取传感器数据
    uart.println("Testing sensor data read...");
    int16_t test_data = read_register_16(MPUREG_ACCEL_XOUT_H);
    uart.print("Test accelerometer reading: ");
    uart.println((int)test_data);
    
    uart.println("MPU6000 initialization completed!");
    return true;
}

bool MPU6000::read_sensors() {
    // 读取加速度计数据
    int16_t accel_x = read_register_16(MPUREG_ACCEL_XOUT_H);
    int16_t accel_y = read_register_16(MPUREG_ACCEL_YOUT_H);
    int16_t accel_z = read_register_16(MPUREG_ACCEL_ZOUT_H);
    
    // 读取温度数据
    int16_t temp = read_register_16(MPUREG_TEMP_OUT_H);
    
    // 读取陀螺仪数据
    int16_t gyro_x = read_register_16(MPUREG_GYRO_XOUT_H);
    int16_t gyro_y = read_register_16(MPUREG_GYRO_YOUT_H);
    int16_t gyro_z = read_register_16(MPUREG_GYRO_ZOUT_H);
    
    // 转换为物理量
    _accel_x = accel_x / 4096.0f;
    _accel_y = accel_y / 4096.0f;
    _accel_z = accel_z / 4096.0f;
    
    _temperature = (temp / 340.0f) + 36.53f;
    
    _gyro_x = gyro_x / 16.4f;
    _gyro_y = gyro_y / 16.4f;
    _gyro_z = gyro_z / 16.4f;
    
    return true;
}