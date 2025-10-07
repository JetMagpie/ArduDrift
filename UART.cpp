#include "UART.h"
#include <util/delay.h>

// 全局UART实例
UART uart;

UART::UART() :
    _rxBuffer{0, 0, 0, NULL},
    _txBuffer{0, 0, 0, NULL},
    _initialized(false),
    _open(false),
    _ubrrh(&UBRR0H),
    _ubrrl(&UBRR0L),
    _ucsra(&UCSR0A),
    _ucsrb(&UCSR0B)
{
}

bool UART::_allocBuffer(Buffer *buffer, uint16_t size) {
    uint8_t mask;
    uint8_t shift;

    // 初始化缓冲区状态
    buffer->head = buffer->tail = 0;

    // 计算大于或等于请求缓冲区大小的2的幂
    for (shift = 1; (1U << shift) < (_max_buffer_size < size ? _max_buffer_size : size); shift++)
        ;
    mask = (1U << shift) - 1;

    // 如果描述符已经分配了缓冲区，我们需要处理它
    if (buffer->bytes) {
        // 如果分配的缓冲区已经是正确的大小，那么我们无事可做
        if (buffer->mask == mask)
            return true;

        // 释放旧缓冲区
        free(buffer->bytes);
    }
    buffer->mask = mask;

    // 为缓冲区分配内存 - 如果失败，我们就失败
    buffer->bytes = (uint8_t *) malloc(buffer->mask + (size_t)1);

    return (buffer->bytes != NULL);
}

void UART::_freeBuffer(Buffer *buffer) {
    buffer->head = buffer->tail = 0;
    buffer->mask = 0;
    if (NULL != buffer->bytes) {
        free(buffer->bytes);
        buffer->bytes = NULL;
    }
}

void UART::begin(uint32_t baudrate) {
    uint16_t ubrr;
    bool use_u2x = true;

    // 如果已经打开，先关闭
    if (_open) {
        *_ucsrb &= ~((1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0));
    }

    // 分配缓冲区
    if (!_allocBuffer(&_rxBuffer, _default_rx_buffer_size) || 
        !_allocBuffer(&_txBuffer, _default_tx_buffer_size)) {
        return; // 无法分配缓冲区 - 致命错误
    }

    // 重置缓冲区指针
    _txBuffer.head = _txBuffer.tail = 0;
    _rxBuffer.head = _rxBuffer.tail = 0;

    // 标记端口为打开
    _open = true;
    _initialized = true;

    // 硬编码异常，用于与Duemilanove和先前板卡附带的引导加载程序以及Uno和Mega 2560上的8U2固件兼容
    #if F_CPU == 16000000UL
        if (baudrate == 57600)
            use_u2x = false;
    #endif

    if (use_u2x) {
        *_ucsra = 1 << U2X0;
        ubrr = (F_CPU / 4 / baudrate - 1) / 2;
    } else {
        *_ucsra = 0;
        ubrr = (F_CPU / 8 / baudrate - 1) / 2;
    }

    *_ubrrh = ubrr >> 8;
    *_ubrrl = ubrr;

    // 启用接收和发送，以及接收中断
    *_ucsrb |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    
    // 等待硬件稳定
    _delay_ms(10);
}

void UART::write(uint8_t c) {
    if (!_open) return;

    uint8_t i = (_txBuffer.head + 1) & _txBuffer.mask;

    // 等待缓冲区有空间
    while (i == _txBuffer.tail);

    // 添加字节到缓冲区
    _txBuffer.bytes[_txBuffer.head] = c;
    _txBuffer.head = i;

    // 启用数据就绪中断（如果缓冲区为空可能关闭）
    *_ucsrb |= (1 << UDRIE0);
}

void UART::write(const uint8_t *buffer, size_t size) {
    if (!_open) return;

    for (size_t i = 0; i < size; i++) {
        write(buffer[i]);
    }
}

void UART::print(const char* str) {
    while (*str) {
        write(*str++);
    }
}

void UART::println(const char* str) {
    print(str);
    write('\r');
    write('\n');
}

void UART::print(float value, uint8_t decimals) {
    if (value < 0) {
        write('-');
        value = -value;
    }
    
    int integer_part = (int)value;
    print(integer_part);
    write('.');
    
    float fractional = value - integer_part;
    for (uint8_t i = 0; i < decimals; i++) {
        fractional *= 10;
        int digit = (int)fractional;
        write('0' + digit);
        fractional -= digit;
    }
}

void UART::println(float value, uint8_t decimals) {
    print(value, decimals);
    write('\r');
    write('\n');
}

void UART::print(int value) {
    if (value == 0) {
        write('0');
        return;
    }
    
    if (value < 0) {
        write('-');
        value = -value;
    }
    
    if (value >= 10) {
        print(value / 10);
    }
    write('0' + (value % 10));
}

void UART::println(int value) {
    print(value);
    write('\r');
    write('\n');
}

void UART::print(uint32_t value) {
    if (value == 0) {
        write('0');
        return;
    }
    
    if (value >= 10) {
        print(value / 10);
    }
    write('0' + (value % 10));
}

void UART::println(uint32_t value) {
    print(value);
    write('\r');
    write('\n');
}

// 新增：uint16_t 支持
void UART::print(uint16_t value) {
    print((uint32_t)value);
}

void UART::println(uint16_t value) {
    print(value);
    write('\r');
    write('\n');
}

// 新增：uint8_t 支持
void UART::print(uint8_t value) {
    print((uint32_t)value);
}

void UART::println(uint8_t value) {
    print(value);
    write('\r');
    write('\n');
}

void UART::printHex(uint8_t value) {
    uint8_t nibble;
    
    // 高4位
    nibble = (value >> 4) & 0x0F;
    write(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
    
    // 低4位
    nibble = value & 0x0F;
    write(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
}

// 新增：16位十六进制支持
void UART::printHex(uint16_t value) {
    printHex((uint8_t)(value >> 8));
    printHex((uint8_t)(value & 0xFF));
}

// 新增：32位十六进制支持  
void UART::printHex(uint32_t value) {
    printHex((uint16_t)(value >> 16));
    printHex((uint16_t)(value & 0xFFFF));
}

int16_t UART::available() {
    if (!_open) return -1;
    return ((_rxBuffer.head - _rxBuffer.tail) & _rxBuffer.mask);
}

int16_t UART::read() {
    uint8_t c;

    // 如果头和尾相等，缓冲区为空
    if (!_open || (_rxBuffer.head == _rxBuffer.tail))
        return -1;

    // 从尾部取出字符
    c = _rxBuffer.bytes[_rxBuffer.tail];
    _rxBuffer.tail = (_rxBuffer.tail + 1) & _rxBuffer.mask;

    return c;
}

void UART::flush() {
    _rxBuffer.head = _rxBuffer.tail;
    _txBuffer.tail = _txBuffer.head;
}

// UART中断处理函数
void UART::rx_isr() {
    uint8_t c = UDR0;
    uint8_t i = (_rxBuffer.head + 1) & _rxBuffer.mask;
    
    if (i != _rxBuffer.tail) {
        _rxBuffer.bytes[_rxBuffer.head] = c;
        _rxBuffer.head = i;
    }
}

void UART::tx_isr() {
    if (_txBuffer.tail != _txBuffer.head) {
        UDR0 = _txBuffer.bytes[_txBuffer.tail];
        _txBuffer.tail = (_txBuffer.tail + 1) & _txBuffer.mask;
    } else {
        // 没有更多字节要发送，禁用中断
        UCSR0B &= ~(1 << UDRIE0);
    }
}

// UART接收中断服务程序
ISR(USART0_RX_vect) {
    uart.rx_isr();
}

// UART数据寄存器空中断服务程序  
ISR(USART0_UDRE_vect) {
    uart.tx_isr();
}