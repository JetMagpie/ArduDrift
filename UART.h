#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

class UART {
public:
    UART();
    void begin(uint32_t baudrate);
    void print(const char* str);
    void println(const char* str);
    void println() { print("\r\n"); }
    void print(float value, uint8_t decimals = 2);
    void println(float value, uint8_t decimals = 2);
    void print(int value);
    void println(int value);
    void print(uint32_t value);
    void println(uint32_t value);
    void print(uint16_t value);      // 添加 uint16_t 支持
    void println(uint16_t value);    // 添加 uint16_t 支持
    void print(uint8_t value);       // 添加 uint8_t 支持  
    void println(uint8_t value);     // 添加 uint8_t 支持
    void printHex(uint8_t value);
    void printHex(uint16_t value);   // 添加 16位十六进制支持
    void printHex(uint32_t value);   // 添加 32位十六进制支持
    
    // 检查是否有数据可读
    int16_t available();
    // 读取一个字节
    int16_t read();
    // 清空缓冲区
    void flush();
    
    // 为中断函数提供公共访问方法
    void rx_isr();
    void tx_isr();
    
    // 添加公共字符输出方法
    void writeChar(char c) { write(c); }
    
private:
    void write(uint8_t c);
    void write(const uint8_t *buffer, size_t size);
    
    // 缓冲区结构
    struct Buffer {
        volatile uint8_t head, tail;
        uint8_t mask;
        uint8_t *bytes;
    };
    
    Buffer _rxBuffer;
    Buffer _txBuffer;
    bool _initialized;
    bool _open;
    
    // 寄存器指针（针对UART0）
    volatile uint8_t * const _ubrrh;
    volatile uint8_t * const _ubrrl;
    volatile uint8_t * const _ucsra;
    volatile uint8_t * const _ucsrb;
    
    // 缓冲区分配函数
    bool _allocBuffer(Buffer *buffer, uint16_t size);
    void _freeBuffer(Buffer *buffer);
    
    static const uint16_t _default_rx_buffer_size = 128;
    static const uint16_t _default_tx_buffer_size = 128;
    static const uint16_t _max_buffer_size = 256;
};

// 声明全局实例
extern UART uart;

#endif // __UART_H__