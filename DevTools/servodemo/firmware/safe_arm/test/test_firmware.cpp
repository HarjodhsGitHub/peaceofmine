#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <new>
#include <tuple>
#include "support/ax12.h"

TestSerial Serial;
unsigned char ax_rx_buffer[32];
uint8_t registers[253][80];
uint32_t clockMs;
void delay(unsigned long ms) { assert(ms == 20); clockMs += ms; }
bool readFailure, corruptRead, ignoreWrites;
bool pendingWriteReply = false;
bool enablePending = false;
uint32_t enableAt = 0;
bool busBusy = false;
void delayMicroseconds(unsigned int us) { assert(us == 1000); busBusy = false; }
std::vector<uint8_t> syncPacket;
std::vector<std::tuple<int, int, int>> writes;
uint32_t millis() { return clockMs; }
void ax12Init(long) {}
int reg(int address) { return registers[1][address] | registers[1][address + 1] << 8; }
void setReg(int address, int value) {
    registers[1][address] = value & 255;
    registers[1][address + 1] = value >> 8;
}
int ax12GetRegister(int id, int address, int size) {
    if (busBusy) return -1;
    busBusy = true;
    if (enablePending && uint32_t(clockMs - enableAt) >= 5) {
        registers[1][24] = 1;
        enablePending = false;
    }
    if (pendingWriteReply) {
        pendingWriteReply = false;
        ax_rx_buffer[2] = id;
        ax_rx_buffer[3] = 2;
        ax_rx_buffer[4] = 0;
        return 0; // Return-level-2 write ACK is not the requested read response.
    }
    if (readFailure || id != 1 || address + size > 80) return -1;
    ax_rx_buffer[2] = id;
    ax_rx_buffer[3] = size + 2;
    ax_rx_buffer[4] = 0;
    std::copy(registers[id] + address, registers[id] + address + size, ax_rx_buffer + 5);
    uint8_t sum = 0;
    for (int i = 2; i < size + 5; ++i) sum += ax_rx_buffer[i];
    ax_rx_buffer[size + 5] = uint8_t(~sum);
    if (corruptRead) { ax_rx_buffer[size + 5] ^= 1; return -1; }
    return int16_t(ax_rx_buffer[5] | (size == 1 ? 0 : ax_rx_buffer[6] << 8));
}
void ax12SetRegister(int id, int address, int value) {
    assert(!busBusy);
    busBusy = true;
    writes.emplace_back(id, address, value);
    if (ignoreWrites) return;
    if (id == 254) { registers[1][address] = value; enablePending = false; return; }
    pendingWriteReply = true;
    assert(id == 1 && address < 80);
    registers[id][address] = value;
}
void ax12SetRegister2(int id, int address, int value) {
    assert(!busBusy);
    busBusy = true;
    writes.emplace_back(id, address, value);
    if (ignoreWrites) return;
    pendingWriteReply = true;
    assert(id == 1 && address < 79);
    setReg(address, value);
}
void setTXall() { assert(!busBusy); busBusy = true; syncPacket.clear(); }
void ax12write(unsigned char value) { syncPacket.push_back(value); }
void setRX(int) {
    assert(syncPacket.size() >= 10);
    assert(syncPacket[0] == 255 && syncPacket[1] == 255 && syncPacket[2] == 254);
    assert(syncPacket[3] + 4u == syncPacket.size() && syncPacket[4] == AX_SYNC_WRITE);
    uint8_t sum = 0;
    for (size_t i = 2; i < syncPacket.size(); ++i) sum += syncPacket[i];
    assert(sum == 255);
    int id = syncPacket[7], address = syncPacket[5], size = syncPacket[6];
    assert(id == 1 && (size == 1 || size == 2));
    int value = syncPacket[8] | (size == 2 ? syncPacket[9] << 8 : 0);
    writes.emplace_back(id, address, value);
    if (!ignoreWrites) {
        if (address == 24 && value == 1) {
            enablePending = true;
            enableAt = clockMs;
        } else registers[id][address] = value & 255;
        if (size == 2) registers[id][address + 1] = value >> 8;
    }
}

// Exercise the actual sketch/parser, not a separate model of its protocol.
#include "../safe_arm.ino"

void reset(uint32_t now = 100) {
    new (&arm) ArmController<ServoIO>(servoIO);
    Serial = TestSerial();
    used = needed = 0;
    clockMs = receivedAt = now;
    std::memset(registers, 0, sizeof(registers));
    std::memset(ax_rx_buffer, 0, sizeof(ax_rx_buffer));
    readFailure = corruptRead = ignoreWrites = false;
    pendingWriteReply = false;
    enablePending = false;
    busBusy = false;
    setReg(0, 310); setReg(8, 4095); setReg(34, 1023); setReg(36, 2000);
    writes.clear();
    setup();
}

std::vector<uint8_t> command(uint8_t id, uint8_t instruction, std::vector<uint8_t> args) {
    std::vector<uint8_t> bytes = {255, 255, id, uint8_t(args.size() + 2), instruction};
    bytes.insert(bytes.end(), args.begin(), args.end());
    uint8_t sum = 0;
    for (size_t i = 2; i < bytes.size(); ++i) sum += bytes[i];
    bytes.push_back(uint8_t(~sum));
    return bytes;
}
void send(const std::vector<uint8_t> &bytes) {
    Serial.outgoing.clear();
    Serial.incoming.insert(Serial.incoming.end(), bytes.begin(), bytes.end());
    while (Serial.available()) loop();
}
bool writeReg(uint8_t id, uint8_t address, uint16_t value, uint8_t size = 1) {
    std::vector<uint8_t> args = {address, uint8_t(value)};
    if (size == 2) args.push_back(value >> 8);
    send(command(id, 3, args));
    return Serial.outgoing.size() == 6 && Serial.outgoing[4] == 0;
}
void start(bool finish = true) {
    assert(writeReg(253, 81, 1));
    assert(writeReg(253, 82, 1));
    assert(writeReg(253, 84, 1500, 2));
    assert(writeReg(253, 86, 2500, 2));
    assert(writeReg(253, 88, 20, 2));
    assert(writeReg(253, 90, 4));
    assert(writeReg(253, 91, 23, 2));
    assert(writeReg(253, 94, 1));
    if (!finish) return;
    for (int i = 0; i < 10 && arm.starting; ++i) {
        clockMs += 20;
        assert(writeReg(253, 82, 2));
    }
    assert(!arm.starting);
    assert(arm.sweeping && registers[1][24] == 1);
}
void heartbeatTick(uint32_t delta = 20) {
    clockMs += delta;
    assert(writeReg(253, 82, 2));
}

int main() {
    reset();
    assert(!writeReg(253, 83, 1)); // Must explicitly select an actuator.
    assert(writeReg(253, 81, 1));
    registers[1][24] = 1;
    assert(!writeReg(253, 83, 1));
    registers[1][24] = 0;
    assert(writeReg(253, 83, 1));
    assert(reg(6) == 4095 && reg(8) == 4095 && registers[1][22] == 1);
    assert(registers[1][24] == 0);
    for (int position : {-28672, -4096, -1, 0, 4096, 28672}) {
        setReg(36, position);
        assert(writeReg(253, 82, 1));
        assert(reg(30) == (position & 65535));
        assert(writeReg(1, 30, uint16_t(position), 2));
        assert(!writeReg(253, 83, 1)); // No EEPROM writes with a live lease.
        assert(!writeReg(1, 30, uint16_t(-28673), 2));
        assert(!writeReg(1, 30, 28673, 2));
        assert(writeReg(1, 24, 1));
        clockMs += 350; loop();
        assert(!arm.permitted && registers[1][24] == 0);
    }
    setReg(36, -1); corruptRead = true;
    assert(!writeReg(253, 82, 1)); // Corrupt 0xffff must not be accepted as -1.
    corruptRead = false; readFailure = true;
    assert(!writeReg(253, 82, 1));
    readFailure = false; registers[1][22] = 2;
    assert(!writeReg(253, 82, 1)); // Wrong encoder scaling.
    reset();
    assert(writeReg(253, 88, 1023, 2));
    assert(arm.speed == 1023);
    assert(!writeReg(253, 88, 1024, 2));
    assert(!writeReg(253, 88, 0, 2));
    reset(); start(false);
    assert(arm.starting && registers[1][24] == 0);
    heartbeatTick(); heartbeatTick();
    assert(writeReg(253, 94, 0));
    clockMs += 300; loop();
    assert(!arm.starting && !arm.sweeping && registers[1][24] == 0);
    reset(); start(false);
    clockMs += 350; loop();
    assert(!arm.starting && !arm.permitted && arm.fault == 2);
    reset(); start(false); ignoreWrites = true;
    for (int i = 0; i < 12 && arm.permitted; ++i) {
        clockMs += 20;
        writeReg(253, 82, 2);
    }
    assert(!arm.permitted && arm.fault == 9 && registers[1][24] == 0);
    reset();
    assert(registers[1][24] == 0);
    assert(!writeReg(1, 24, 1));
    assert(!writeReg(1, 30, 2200, 2));
    assert(!writeReg(253, 94, 1));
    start();
    uint32_t deadline = clockMs + 350;
    clockMs = deadline - 1; loop();
    assert(arm.permitted);
    clockMs = deadline;
    assert(!writeReg(253, 82, 2)); // Too-late heartbeat cannot re-arm.
    assert(!arm.permitted && !arm.sweeping && arm.fault == 2 && registers[1][24] == 0);
    assert(!writeReg(253, 94, 1));
    assert(!writeReg(1, 24, 1));
    assert(writeReg(253, 82, 1)); // Explicit new permission, holding present position.
    assert(reg(30) == reg(36) && registers[1][24] == 0 && !arm.sweeping);

    reset(UINT32_MAX - 100); start();
    clockMs += 350; loop();
    assert(!arm.permitted && registers[1][24] == 0);

    reset(); start();
    for (int i = 0; i < 20; ++i) {
        clockMs += 20;
        send(command(1, 2, {36, 2})); // Telemetry cannot extend permission.
    }
    assert(!arm.permitted);

    reset(); start();
    auto badHeartbeat = command(253, 3, {82, 2});
    badHeartbeat.back() ^= 1;
    for (int i = 0; i < 20; ++i) { clockMs += 20; send(badHeartbeat); }
    assert(!arm.permitted);

    reset(); start();
    for (int i = 0; i < 10; ++i) {
        auto before = writes.size();
        assert(writeReg(253, 94, 1)); // No restart, no extra motor writes.
        assert(writes.size() == before);
    }
    assert(!writeReg(253, 84, 1000, 2));
    assert(!writeReg(253, 88, 0, 2));
    assert(!writeReg(253, 90, 0));
    assert(!writeReg(1, 24, 1));
    assert(!writeReg(1, 30, 2400, 2));
    assert(!writeReg(1, 6, 0, 2)); // No EEPROM writes.
    send(command(254, 3, {24, 1}));
    assert(Serial.outgoing.empty());
    assert(writeReg(253, 88, 10, 2));
    assert(arm.sweeping && registers[1][24] == 1 && arm.speed == 10);

    reset(); start();
    for (int i = 0; i < 20; ++i) heartbeatTick();
    assert(reg(32) == 20);
    uint16_t lastSpeed = reg(32);
    for (int position = 2360; position <= 2476; position += 4) {
        setReg(36, position);
        heartbeatTick();
        assert(reg(32) <= lastSpeed && reg(32) >= 1);
        lastSpeed = reg(32);
    }
    assert(reg(32) > 1); // No minimum-speed crawl outside the endpoint band.
    setReg(36, 2478); setReg(38, 10);
    heartbeatTick();
    assert(arm.settling && reg(30) == 2478);
    for (int i = 0; i < 20; ++i) heartbeatTick();
    assert(arm.settling && reg(30) == 2478); // No reversal while still moving.
    setReg(38, 0);
    for (int i = 0; i < 10; ++i) heartbeatTick();
    assert(!arm.settling && reg(30) == 1500 && reg(32) == 1);
    assert(writeReg(1, 24, 0)); // Kill bypasses smooth corner handling.
    assert(!arm.sweeping && registers[1][24] == 0);

    reset(); start();
    setReg(36, 2478); setReg(38, 3); heartbeatTick();
    for (int i = 0; i < 10; ++i) {
        setReg(36, 2478 + i % 2);
        heartbeatTick();
    }
    assert(!arm.settling && arm.goal == 1500 && arm.fault == 0);

    reset(); start();
    setReg(36, 2478); setReg(38, 0); heartbeatTick();
    for (int i = 0; i < 74; ++i) {
        setReg(36, 2478 + (i % 2) * 4);
        heartbeatTick();
    }
    clockMs += 20; loop();
    assert(arm.fault == 10 && !arm.permitted && registers[1][24] == 0);

    reset(); start();
    for (int i = 0; i < 150; ++i) heartbeatTick();
    assert(arm.fault == 0 && arm.permitted); // No position-only stall inference.
    clockMs += 350; loop();
    assert(arm.fault == 2 && !arm.permitted && registers[1][24] == 0);
    reset(); start(); readFailure = true; clockMs += 20; loop();
    assert(!arm.permitted && arm.fault == 3);
    reset(); start(); setReg(36, 1499); clockMs += 20; loop();
    assert(!arm.permitted && arm.fault == 3);
    reset(); start(); setReg(34, 0); clockMs += 20; loop();
    assert(!arm.permitted && arm.fault == 4);
    reset(); start(); registers[1][24] = 0; clockMs += 20; loop();
    assert(!arm.permitted && arm.fault == 4);
    reset(); corruptRead = true;
    send(command(1, 2, {36, 2}));
    assert(Serial.outgoing.size() == 6 && Serial.outgoing[4] != 0);

    reset(); start();
    setReg(36, 2480); heartbeatTick();
    assert(arm.settling);
    clockMs += 350; loop();
    assert(!arm.permitted && !arm.settling && registers[1][24] == 0);

    reset();
    send({255, 255, 253, 255}); // Oversize length and truncated packet recovery.
    send({255, 255, 253, 5, 3});
    clockMs += 21;
    assert(writeReg(253, 81, 1));
    uint32_t random = 123;
    for (int i = 0; i < 10000; ++i) {
        random = random * 1664525 + 1013904223;
        clockMs += 1;
        send({uint8_t(random >> 24)});
        assert(!arm.permitted);
    }
    clockMs += 21;
    assert(writeReg(253, 81, 1));

    // Kinematic fake servo exercises many complete forward/reverse cycles.
    reset(); start();
    double position = 2000;
    int reversals = 0, previousEndpoint = arm.goal;
    for (int i = 0; i < 8000; ++i) {
        double distance = reg(30) - position;
        double travel = reg(32) * .684 * 4096 / 360 * .020;
        double step = std::min(std::abs(distance), travel) * (distance < 0 ? -1 : 1);
        position += step;
        setReg(36, int(std::round(position)));
        setReg(38, std::abs(distance) <= travel ? 0 : reg(32));
        heartbeatTick();
        if (i % 10 == 0) {
            assert(writeReg(253, 88, (i / 10) % 2 ? 10 : 20, 2));
            send(command(1, 2, {24, 23}));
            assert(Serial.outgoing.size() == 29 && Serial.outgoing[4] == 0);
        }
        assert(position >= 1500 && position <= 2500);
        assert(reg(32) >= 1 && reg(32) <= 20);
        if (!arm.settling && arm.goal != previousEndpoint) {
            ++reversals;
            previousEndpoint = arm.goal;
        }
    }
    assert(reversals >= 8);
    std::cout << "Firmware parser, watchdog, faults, smooth corners and " << reversals
              << " simulated reversals passed\n";
}
