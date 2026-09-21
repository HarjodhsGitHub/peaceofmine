#pragma once
#include <stdint.h>
#include <deque>
#include <vector>

struct TestSerial {
    std::deque<uint8_t> incoming;
    std::vector<uint8_t> outgoing;
    void begin(long) {}
    int available() { return incoming.size(); }
    int read() { int value = incoming.front(); incoming.pop_front(); return value; }
    void write(uint8_t value) { outgoing.push_back(value); }
};
extern TestSerial Serial;
extern unsigned char ax_rx_buffer[32];
uint32_t millis();
void delayMicroseconds(unsigned int us);
void ax12Init(long);
int ax12GetRegister(int id, int address, int size);
void ax12SetRegister(int id, int address, int value);
void ax12SetRegister2(int id, int address, int value);
#define AX_SYNC_WRITE 0x83
void setTXall();
void ax12write(unsigned char value);
void setRX(int id);
