#ifndef __PWM_OUTPUT_H__
#define __PWM_OUTPUT_H__

#include <stdint.h>
#include <avr/io.h>

class PWMOutput {
public:
    // 初始化PWM输出系统
    static void init();
    
    // 设置指定通道的PWM脉冲宽度 (1000-2000微秒)
    static void setPulse(uint8_t channel, uint16_t pulse_us);
    
    // 读取指定通道的当前PWM值
    static uint16_t read(uint8_t channel);
    
    // 启用指定通道
    static void enable(uint8_t channel);
    
    // 禁用指定通道
    static void disable(uint8_t channel);
    
    // 设置所有通道的频率 (通常50Hz用于舵机)
    static void setFrequency(uint16_t freq_hz);
    
    // 通道定义 (基于APM2引脚映射)
    enum Channels {
        CH1 = 0,   // 引脚11
        CH2 = 1,   // 引脚12  
        CH3 = 2,   // 引脚8
        CH4 = 3,   // 引脚7
        CH5 = 4,   // 引脚6
        CH6 = 5,   // 引脚5
        CH7 = 6,   // 引脚3
        CH8 = 7,   // 引脚2
        CH10 = 9,  // 引脚45
        CH11 = 10  // 引脚44
    };

private:
    // 计算定时器周期值
    static uint16_t calculatePeriod(uint16_t freq_hz);
    
    // 约束脉冲宽度
    static uint16_t constrainPulse(uint16_t pulse_us);
};

#endif // __PWM_OUTPUT_H__