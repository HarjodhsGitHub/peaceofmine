#ifndef PEACEOFMINE_ARM_CONTROLLER_H
#define PEACEOFMINE_ARM_CONTROLLER_H
#include <stdint.h>
#include <math.h>

// IO supplies read(), readMotion(), write() and writeEeprom().
template<class IO> class ArmController {
public:
    static const uint32_t timeoutMs = 350;
    IO &io;
    uint8_t selected, fault;
    bool permitted, sweeping, testBounds;
    uint16_t minimum, maximum, speed, tolerance, goal, writtenGoal;
    uint8_t acceleration;
    uint32_t heartbeat, lastPoll, lastOff;
    bool settling;
    uint32_t cornerAt;
    bool starting;
    uint8_t startupStage;
    uint32_t startupAt;
    uint32_t legAt, legTimeoutMs;
    float rampedSpeed;
    uint16_t appliedSpeed;

    explicit ArmController(IO &port): io(port), selected(255), fault(0),
        permitted(false), sweeping(false), testBounds(false), minimum(0), maximum(0), speed(10),
        tolerance(23), goal(0), writtenGoal(0), acceleration(4), heartbeat(0), lastPoll(0),
        lastOff(0), settling(false), cornerAt(0),
        starting(false), startupStage(0), startupAt(0), legAt(0), legTimeoutMs(0), rampedSpeed(1), appliedSpeed(1) {}

    void stop(uint8_t reason = 0) {
        permitted = sweeping = false;
        settling = false;
        starting = false;
        fault = reason;
        // This firmware owns the whole attached actuator bus.
        io.write(254, 24, 0, 1);
    }

    void beginLeg(int position, uint16_t endpoint, uint32_t now) {
        goal = writtenGoal = endpoint;
        legAt = now;
        float degrees = fabsf(float(endpoint)-position)*(360.0f/4096);
        // Conservative travel deadline, including acceleration/braking and
        // settling. A stalled mechanism must not run indefinitely.
        legTimeoutMs = uint32_t(ceilf(2000*(degrees/(speed*.684f)+speed*.684f/(acceleration*8.583f))))+1500;
        rampedSpeed = appliedSpeed = 1;
        io.write(selected, 32, 1, 2);
        io.write(selected, 73, acceleration, 1);
        io.write(selected, 30, endpoint, 2);
        settling = false;
    }

    void tick(uint32_t now) {
        if (permitted && uint32_t(now - heartbeat) >= timeoutMs) stop(2);
        if (!permitted) {
            // Also catch an actuator which powers up after the controller.
            if (uint32_t(now - lastOff) >= 100) {
                io.write(254, 24, 0, 1);
                lastOff = now;
            }
            return;
        }
        if (testBounds && !sweeping && uint32_t(now-lastPoll) >= 10) {
            lastPoll = now;
            int position = io.read(selected, 36, 2);
            if (position < int(minimum) || position > int(maximum)) stop(3);
        }
        if (!permitted) return;
        if (!sweeping || uint32_t(now - lastPoll) < (starting ? 20u : 10u)) return;
        uint32_t elapsed = now-lastPoll;
        if (elapsed > 100) elapsed = 100;
        lastPoll = now;
        int position, velocity = 0, torque = 0, torqueLimit = 0;
        if (starting) position = io.read(selected, 36, 2);
        else if (!io.readMotion(selected, position, velocity, torque, torqueLimit)) {
            stop(3); return;
        }
        if (position < int(minimum) || position > int(maximum)) { stop(3); return; }
        if (starting) {
            // Stage startup across ticks. A servo need not apply RAM writes
            // before a read immediately following them on the bus.
            if (uint32_t(now - startupAt) >= 250) { stop(9); return; }
            switch (startupStage) {
            case 0: io.write(selected, 32, 1, 2); break;
            case 1: io.write(selected, 73, acceleration, 1); break;
            case 2: goal = writtenGoal = position; io.write(selected, 30, goal, 2); break;
            case 3:
                if (io.read(selected, 32, 2) != 1 || io.read(selected, 73, 1) != acceleration ||
                    io.read(selected, 30, 2) != int(goal)) {
                    stop(9); return;
                }
                io.write(selected, 24, 1, 1);
                break;
            case 4:
                if (io.read(selected, 24, 1) != 1) return;
                beginLeg(position, position >= int(maximum - tolerance) ? minimum : maximum, now);
                starting = false;
                break;
            }
            ++startupStage;
            return;
        }
        if (torque != 1 || torqueLimit <= 0) { stop(4); return; }
        if (uint32_t(now-legAt) >= legTimeoutMs) { stop(10); return; }
        int distance = position-int(goal);
        if (distance < 0) distance = -distance;
        // Keep the real endpoint as the servo's target. Streaming nearby
        // intermediate targets left this motor hunting behind the reference.
        float remaining = distance-int(tolerance);
        if (remaining < 0) remaining = 0;
        float desired = sqrtf(2*acceleration*8.583f*remaining*(360.0f/4096))/.684f;
        if (desired > speed) desired = speed;
        if (desired < 1) desired = 1; // Register zero means unlimited speed.
        float delta = acceleration*(8.583f/.684f)*elapsed/1000;
        if (rampedSpeed < desired) {
            rampedSpeed += delta;
            if (rampedSpeed > desired) rampedSpeed = desired;
        } else {
            rampedSpeed -= delta;
            if (rampedSpeed < desired) rampedSpeed = desired;
        }
        uint16_t limited = uint16_t(rampedSpeed+.5f);
        if (limited < 1) limited = 1;
        if (limited != appliedSpeed) {
            io.write(selected, 32, limited, 2);
            appliedSpeed = limited;
        }
        if (distance <= int(tolerance)) {
            if (!settling) { settling = true; cornerAt = now; }
            if ((velocity & 1023) <= 3) {
                beginLeg(position, goal == maximum ? minimum : maximum, now);
            } else if (uint32_t(now-cornerAt) >= 1500) stop(10);
        } else if (settling && uint32_t(now-cornerAt) >= 1500) stop(10);
    }

    bool lease(uint8_t action, uint32_t now) {
        tick(now);
        if (action == 0) { stop(); return true; }
        if (action == 2) {
            if (!permitted) return false;
            heartbeat = now;
            return true;
        }
        if (action != 1 || permitted || selected == 255) return false;
        if (io.read(selected, 0, 2) != 310 || io.read(selected, 70, 1) != 0 ||
            io.read(selected, 24, 1) != 0) return false;
        int cw = io.read(selected, 6, 2), ccw = io.read(selected, 8, 2);
        int32_t position = io.read(selected, 36, 2);
        if (position < 0 || !validPosition(selected, cw, ccw, uint16_t(position))) return false;
        if (testBounds && (minimum >= maximum || position < minimum || position > maximum)) return false;
        // Re-arming never replays the goal left behind by a previous session.
        io.write(selected, 30, position, 2);
        io.write(selected, 32, 1, 2);
        heartbeat = now;
        permitted = true;
        fault = 0;
        return true;
    }

    bool validPosition(uint8_t id, int cw, int ccw, uint16_t raw) {
        if (cw == 4095 && ccw == 4095) {
            // Decode explicitly: AVR int is 16-bit, native test int is 32-bit.
            int32_t position = raw >= 32768 ? int32_t(raw) - 65536L : int32_t(raw);
            return io.read(id, 22, 1) == 1 && position >= -28672L && position <= 28672L;
        }
        return cw >= 0 && ccw > cw && ccw <= 4095 && raw <= 4095 && int(raw) >= cw && int(raw) <= ccw;
    }

    bool start(uint32_t now) {
        tick(now);
        if (permitted && sweeping) return true; // Duplicate start is not a restart or heartbeat.
        if (!permitted) return false;
        if (minimum >= maximum || maximum > 4095 ||
            speed < 1 || speed > 1023 || acceleration < 1 || acceleration > 254 ||
            tolerance == 0 || tolerance > (maximum - minimum) / 4) { stop(6); return false; }
        int cw = io.read(selected, 6, 2), ccw = io.read(selected, 8, 2);
        int position = io.read(selected, 36, 2);
        if (cw < 0 || ccw <= cw || ccw > 4095 || int(minimum) < cw || int(maximum) > ccw ||
            position < int(minimum) || position > int(maximum)) { stop(7); return false; }
        if (io.read(selected, 34, 2) <= 0) { stop(8); return false; }
        settling = false;
        starting = true;
        startupStage = 0;
        startupAt = now;
        sweeping = true;
        lastPoll = now;
        return true;
    }

    bool configure(uint8_t address, uint16_t value, uint8_t size) {
        if (sweeping) {
            if (address == 84 && size == 2) return value == minimum;
            if (address == 86 && size == 2) return value == maximum;
            if (address == 91 && size == 2) return value == tolerance;
            if (address != 88 && address != 90) return false;
        }
        if (testBounds && permitted && (address == 84 || address == 86)) return false;
        switch (address) {
        case 95:
            if (permitted || selected == 255 || size != 1 || value > 1 ||
                (value && minimum >= maximum)) return false;
            testBounds = value;
            return true;
        case 83:
            // Explicit stopped-only setup. Never enable torque or rewrite PID.
            if (permitted || selected == 255 || size != 1 || value != 1 ||
                io.read(selected, 0, 2) != 310 || io.read(selected, 24, 1) != 0 ||
                io.read(selected, 70, 1) != 0) return false;
            io.writeEeprom(selected, 22, 1, 1);
            io.writeEeprom(selected, 8, 4095, 2);
            io.writeEeprom(selected, 6, 4095, 2);
            return true; // Host polls EEPROM readback before exposing motion.
        case 81:
            if (permitted || size != 1 || value > 252) return false;
            selected = value;
            minimum = maximum = 0;
            testBounds = false;
            return true;
        case 84: if (size != 2 || value > 4095) return false; minimum = value; return true;
        case 86: if (size != 2 || value > 4095) return false; maximum = value; return true;
        case 88:
            if (size != 2 || value < 1 || value > 1023) return false;
            if (sweeping && value < speed)
                legTimeoutMs = uint32_t(ceilf(float(legTimeoutMs)*speed/value));
            speed = value; return true;
        case 90:
            if (size != 1 || value < 1 || value > 254) return false;
            acceleration = value;
            return true;
        case 91: if (size != 2 || value < 1 || value > 1023) return false; tolerance = value; return true;
        default: return false;
        }
    }

    bool writeServo(uint8_t id, uint8_t address, uint16_t value, uint8_t size, uint32_t now) {
        tick(now);
        if (address == 24 && size == 1 && value == 0) { stop(); return true; }
        // PID calibration is RAM-only and requires a stopped, selected MX-64.
        if (address >= 26 && address <= 28) {
            if (permitted || id != selected || size != 1 || value > 254 ||
                io.read(id, 0, 2) != 310 || io.read(id, 24, 1) != 0) return false;
            io.write(id, address, value, size);
            return true;
        }
        if (!permitted || sweeping || id != selected) return false;
        bool valid = false;
        if (address == 24) valid = size == 1 && value == 1 && io.read(id, 34, 2) > 0;
        if (address == 32) valid = size == 2 && value <= 1023;
        if (address == 73) valid = size == 1 && value <= 254;
        if (address == 30 && size == 2) {
            int cw = io.read(id, 6, 2), ccw = io.read(id, 8, 2);
            valid = validPosition(id, cw, ccw, value) && (!testBounds || (value >= minimum && value <= maximum));
        }
        if (!valid) return false;
        io.write(id, address, value, size);
        return true;
    }
};
#endif
