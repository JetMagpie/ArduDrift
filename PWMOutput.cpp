#include "PWMOutput.h"
#include <avr/interrupt.h>

// 初始化所有PWM定时器
void PWMOutput::init() {
    // ... 保持之前的init代码不变 ...
    uint8_t oldSREG = SREG;
    cli();
    
    // TIMER1: CH1 and CH2
    DDRB |= (1 << DDB5) | (1 << DDB6);
    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 40000;
    OCR1A = 0xFFFF;
    OCR1B = 0xFFFF;
    
    // TIMER4: CH3, CH4, CH5
    DDRH |= (1 << DDH3) | (1 << DDH4) | (1 << DDH5);
    TCCR4A = (1 << WGM41);
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41);
    ICR4 = 40000;
    OCR4A = 0xFFFF;
    OCR4B = 0xFFFF;
    OCR4C = 0xFFFF;
    
    // TIMER3: CH6, CH7, CH8
    DDRE |= (1 << DDE3) | (1 << DDE4) | (1 << DDE5);
    TCCR3A = (1 << WGM31);
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31);
    ICR3 = 40000;
    OCR3A = 0xFFFF;
    OCR3B = 0xFFFF;
    OCR3C = 0xFFFF;
    
    // TIMER5: CH10, CH11
    DDRL |= (1 << DDL4) | (1 << DDL5);
    TCCR5A = (1 << WGM51);
    TCCR5B = (1 << WGM53) | (1 << WGM52) | (1 << CS51);
    OCR5A = 40000;
    OCR5B = 0xFFFF;
    OCR5C = 0xFFFF;
    
    SREG = oldSREG;
}

void PWMOutput::setPulse(uint8_t channel, uint16_t pulse_us) {
    pulse_us = constrainPulse(pulse_us);
    uint16_t pwm_value = pulse_us << 1; // 转换为0.5us单位
    
    switch(channel) {
        case CH1:  OCR1B = pwm_value; break;
        case CH2:  OCR1A = pwm_value; break;
        case CH3:  OCR4C = pwm_value; break;
        case CH4:  OCR4B = pwm_value; break;
        case CH5:  OCR4A = pwm_value; break;
        case CH6:  OCR3C = pwm_value; break;
        case CH7:  OCR3B = pwm_value; break;
        case CH8:  OCR3A = pwm_value; break;
        case CH10: OCR5B = pwm_value; break;
        case CH11: OCR5C = pwm_value; break;
    }
}

// 新增的read方法 - 读取指定通道的当前PWM值
uint16_t PWMOutput::read(uint8_t channel) {
    uint16_t pwm_value = 0;
    
    switch(channel) {
        case CH1:  pwm_value = OCR1B; break;
        case CH2:  pwm_value = OCR1A; break;
        case CH3:  pwm_value = OCR4C; break;
        case CH4:  pwm_value = OCR4B; break;
        case CH5:  pwm_value = OCR4A; break;
        case CH6:  pwm_value = OCR3C; break;
        case CH7:  pwm_value = OCR3B; break;
        case CH8:  pwm_value = OCR3A; break;
        case CH10: pwm_value = OCR5B; break;
        case CH11: pwm_value = OCR5C; break;
        default:   return 0;
    }
    
    // 将0.5us单位转换回1us单位并约束范围
    return constrainPulse(pwm_value >> 1);
}

void PWMOutput::enable(uint8_t channel) {
    switch(channel) {
        case CH1:  TCCR1A |= (1 << COM1B1); break;
        case CH2:  TCCR1A |= (1 << COM1A1); break;
        case CH3:  TCCR4A |= (1 << COM4C1); break;
        case CH4:  TCCR4A |= (1 << COM4B1); break;
        case CH5:  TCCR4A |= (1 << COM4A1); break;
        case CH6:  TCCR3A |= (1 << COM3C1); break;
        case CH7:  TCCR3A |= (1 << COM3B1); break;
        case CH8:  TCCR3A |= (1 << COM3A1); break;
        case CH10: TCCR5A |= (1 << COM5B1); break;
        case CH11: TCCR5A |= (1 << COM5C1); break;
    }
}

void PWMOutput::disable(uint8_t channel) {
    switch(channel) {
        case CH1:  TCCR1A &= ~(1 << COM1B1); break;
        case CH2:  TCCR1A &= ~(1 << COM1A1); break;
        case CH3:  TCCR4A &= ~(1 << COM4C1); break;
        case CH4:  TCCR4A &= ~(1 << COM4B1); break;
        case CH5:  TCCR4A &= ~(1 << COM4A1); break;
        case CH6:  TCCR3A &= ~(1 << COM3C1); break;
        case CH7:  TCCR3A &= ~(1 << COM3B1); break;
        case CH8:  TCCR3A &= ~(1 << COM3A1); break;
        case CH10: TCCR5A &= ~(1 << COM5B1); break;
        case CH11: TCCR5A &= ~(1 << COM5C1); break;
    }
}

void PWMOutput::setFrequency(uint16_t freq_hz) {
    uint16_t period = calculatePeriod(freq_hz);
    
    // 更新所有定时器的TOP值
    ICR1 = period;
    ICR3 = period;
    ICR4 = period;
    OCR5A = period; // 定时器5使用OCR5A作为TOP
}

uint16_t PWMOutput::calculatePeriod(uint16_t freq_hz) {
    return 2000000UL / freq_hz; // 8预分频，0.5us精度
}

uint16_t PWMOutput::constrainPulse(uint16_t pulse_us) {
    if (pulse_us < 1000) return 1000;
    if (pulse_us > 2000) return 2000;
    return pulse_us;
}