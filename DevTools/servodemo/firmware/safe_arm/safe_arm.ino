// Dedicated MX-64 arm gateway. The legacy base/pose commands are not exposed.
#include <ax12.h>
#include "controller.h"

struct ServoIO {
    bool readPacket(uint8_t id, uint8_t address, uint8_t size) {
        // Leave a bounded inter-packet processing gap, including after sync
        // writes. Both sweep control and USB telemetry must use this path.
        delayMicroseconds(1000);
        for (uint8_t i = 0; i < size + 6 && i < 32; ++i) ax_rx_buffer[i] = 0;
        int value = ax12GetRegister(id, address, size);
        // 0xffff is the valid multi-turn position -1, as well as the old
        // library's error sentinel on AVR. Only accept it with a valid packet.
        uint8_t sum = 0;
        if (size <= 24) for (uint8_t i = 2; i < size + 6; ++i) sum += ax_rx_buffer[i];
        bool minusOne = size == 2 && ax_rx_buffer[5] == 255 && ax_rx_buffer[6] == 255 && sum == 255;
        return (value != -1 || minusOne) && ax_rx_buffer[2] == id && ax_rx_buffer[3] == size + 2;
    }
    int32_t read(uint8_t id, uint8_t address, uint8_t size) {
        if (!readPacket(id, address, size) || ax_rx_buffer[4] != 0) return -1;
        return uint32_t(ax_rx_buffer[5]) | (size == 2 ? uint32_t(ax_rx_buffer[6]) << 8 : 0);
    }
    void write(uint8_t id, uint8_t address, uint16_t value, uint8_t size) {
        delayMicroseconds(1000);
        if (id == 254) {
            ax12SetRegister(id, address, value);
            return;
        }
        // Match BioloidController::writePose: a one-servo sync write has no
        // status reply, even at return level 2. Unconsumed write replies can
        // collide with the next startup write/read on this half-duplex bus.
        uint8_t length = size + 5;
        uint8_t sum = 254 + length + AX_SYNC_WRITE + address + size + id;
        setTXall();
        ax12write(255); ax12write(255); ax12write(254);
        ax12write(length); ax12write(AX_SYNC_WRITE);
        ax12write(address); ax12write(size); ax12write(id);
        ax12write(uint8_t(value)); sum += uint8_t(value);
        if (size == 2) { ax12write(uint8_t(value >> 8)); sum += uint8_t(value >> 8); }
        ax12write(uint8_t(~sum));
        setRX(0);
    }
    void writeEeprom(uint8_t id, uint8_t address, uint16_t value, uint8_t size) {
        write(id, address, value, size);
        delay(20); // Stopped-only configuration; allow EEPROM programming to finish.
    }
} servoIO;
ArmController<ServoIO> arm(servoIO);

uint8_t packet[40], used = 0, needed = 0;
uint32_t receivedAt = 0;

void reply(uint8_t id, uint8_t error, const uint8_t *data = 0, uint8_t count = 0) {
    uint8_t sum = id + count + 2 + error;
    Serial.write(0xff); Serial.write(0xff);
    Serial.write(id); Serial.write(uint8_t(count + 2)); Serial.write(error);
    for (uint8_t i = 0; i < count; ++i) { Serial.write(data[i]); sum += data[i]; }
    Serial.write(uint8_t(~sum));
}

void dispatch(uint32_t now) {
    uint8_t id = packet[2], length = packet[3], instruction = packet[4];
    uint8_t *args = packet + 5;
    arm.tick(now);
    if (id > 253) return; // No broadcast/sync-write bypass of the safety gate.
    if (instruction == 2 && length == 4) {
        uint8_t address = args[0], count = args[1];
        if (!count || count > 24 || uint16_t(address) + count > 256) { reply(id, 8); return; }
        if (id == 253) {
            if (count != 1) { reply(id, 8); return; }
            uint8_t value;
            switch (address) {
            case 0: value = 44; break;
            case 2: case 80: value = 2; break;
            case 81: value = arm.selected; break;
            case 96: value = arm.fault ? 3 : arm.sweeping ? 2 : arm.permitted ? 1 : 0; break;
            case 98: value = arm.fault; break;
            case 99: value = 0; break; // No independent wired kill input.
            default: reply(id, 8); return;
            }
            reply(id, 0, &value, 1);
        } else {
            if (!servoIO.readPacket(id, address, count)) { reply(id, 32); return; }
            reply(id, ax_rx_buffer[4], ax_rx_buffer + 5, count);
        }
        return;
    }
    if (instruction != 3 || (length != 4 && length != 5)) { reply(id, 64); return; }
    uint8_t address = args[0], size = length - 3;
    uint16_t value = args[1];
    if (size == 2) value |= uint16_t(args[2]) << 8;
    bool ok = false;
    if (id == 253) {
        if (address == 82 && size == 1) ok = arm.lease(value, now);
        else if (address == 94 && size == 1 && value == 1) ok = arm.start(now);
        else if (address == 94 && size == 1 && value == 0) { arm.stop(); ok = true; }
        else ok = arm.configure(address, value, size);
    } else ok = arm.writeServo(id, address, value, size, now);
    reply(id, ok ? 0 : 8);
}

void setup() {
    ax12Init(1000000);
    arm.stop();
    Serial.begin(115200);
}

void loop() {
    arm.tick(millis());
    // Bound parser work so even continuous garbage cannot starve the watchdog.
    for (uint8_t budget = 0; budget < 32 && Serial.available(); ++budget) {
        uint32_t now = millis();
        if (used && uint32_t(now - receivedAt) >= 20) used = 0;
        receivedAt = now;
        uint8_t value = Serial.read();
        if (used < 2) {
            used = value == 255 ? used + 1 : 0;
            if (used) packet[used - 1] = value;
            continue;
        }
        if (used == 2 && value == 255) continue;
        packet[used++] = value;
        if (used == 4) {
            if (value < 2 || value > sizeof(packet) - 4) { used = 0; continue; }
            needed = value + 4;
        }
        if (used >= 4 && used == needed) {
            uint8_t sum = 0;
            for (uint8_t i = 2; i < used; ++i) sum += packet[i];
            if (sum == 255) dispatch(now);
            used = 0;
        }
        arm.tick(millis());
    }
}
