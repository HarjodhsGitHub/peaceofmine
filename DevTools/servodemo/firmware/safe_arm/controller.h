#ifndef PEACEOFMINE_ARM_CONTROLLER_H
#define PEACEOFMINE_ARM_CONTROLLER_H
#include <stdint.h>
#include <math.h>

// IO supplies read(id, address, size) and write(id, address, value, size).
template<class IO> class ArmController {
public:
    static const uint32_t timeoutMs = 350;
    IO &io;
    uint8_t selected, fault;
    bool permitted, sweeping;
    uint16_t minimum, maximum, speed, tolerance, goal;
    uint8_t acceleration;
    uint32_t heartbeat, lastPoll, lastOff, movementAt;
    int lastPosition;
    uint16_t appliedSpeed;
    float rampedSpeed;
    bool settling, nextMaximum;
    uint32_t settledAt, cornerAt;
    bool starting;
    uint8_t startupStage;
    uint32_t startupAt;

    explicit ArmController(IO &port): io(port), selected(255), fault(0),
        permitted(false), sweeping(false), minimum(0), maximum(0), speed(10),
        tolerance(23), goal(0), acceleration(4), heartbeat(0), lastPoll(0),
        lastOff(0), movementAt(0), lastPosition(-1), appliedSpeed(1),
        rampedSpeed(1), settling(false), nextMaximum(false), settledAt(0), cornerAt(0),
        starting(false), startupStage(0), startupAt(0) {}

    void stop(uint8_t reason = 0) {
        permitted = sweeping = false;
        settling = false;
        starting = false;
        fault = reason;
        // This firmware owns the whole attached actuator bus.
        io.write(254, 24, 0, 1);
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
        if (!sweeping || uint32_t(now - lastPoll) < 20) return;
        uint32_t elapsed = now - lastPoll;
        if (elapsed > 100) elapsed = 100;
        lastPoll = now;
        int position = io.read(selected, 36, 2);
        if (position < int(minimum) || position > int(maximum)) { stop(3); return; }
        if (starting) {
            // Stage startup across ticks. A servo need not apply RAM writes
            // before a read immediately following them on the bus.
            if (uint32_t(now - startupAt) >= 250) { stop(9); return; }
            switch (startupStage) {
            case 0: io.write(selected, 32, 1, 2); break;
            case 1: io.write(selected, 73, acceleration, 1); break;
            case 2: goal = position; io.write(selected, 30, goal, 2); break;
            case 3:
                if (io.read(selected, 32, 2) != 1 || io.read(selected, 73, 1) != acceleration ||
                    io.read(selected, 30, 2) != int(goal)) {
                    stop(9); return;
                }
                io.write(selected, 24, 1, 1);
                break;
            case 4:
                if (io.read(selected, 24, 1) != 1) return;
                goal = position >= int(maximum - tolerance) ? minimum : maximum;
                io.write(selected, 30, goal, 2);
                starting = false;
                movementAt = now;
                lastPosition = position;
                break;
            }
            ++startupStage;
            return;
        }
        if (io.read(selected, 24, 1) != 1 || io.read(selected, 34, 2) <= 0) {
            stop(4); return;
        }
        if (settling) {
            int velocity = io.read(selected, 38, 2);
            if (velocity < 0) { stop(3); return; }
            // Allow low-speed telemetry quantization, but require the position
            // to stay within two encoder ticks throughout the settling window.
            if ((velocity & 1023) > 3 || position - lastPosition > 2 || lastPosition - position > 2) {
                settledAt = now;
                lastPosition = position;
            }
            if (uint32_t(now - cornerAt) >= 1500) { stop(10); return; }
            if (uint32_t(now - settledAt) >= 200) {
                goal = nextMaximum ? maximum : minimum;
                io.write(selected, 30, goal, 2);
                settling = false;
                movementAt = now;
                lastPosition = position;
            }
            return;
        }
        if (lastPosition < 0 || position - lastPosition >= 3 || lastPosition - position >= 3) {
            movementAt = now;
            lastPosition = position;
        }
        // Encoder displacement alone is not a reliable overload detector.
        // Permission expiry, servo shutdown and endpoint settling remain gates.
        int distance = int(goal) > position ? int(goal) - position : position - int(goal);
        // Brake by remaining distance. An additional smoothstep envelope used
        // to force minimum-speed crawling well before entering the endpoint.
        // Register zero means unlimited, so the lower bound stays one.
        float desired = speed;
        float remaining = distance - int(tolerance);
        if (remaining < 0) remaining = 0;
        float brakingSpeed = sqrtf(2 * acceleration * 8.583f * remaining * (360.0f / 4096)) / .684f;
        if (brakingSpeed < 1) brakingSpeed = 1;
        if (desired > brakingSpeed) desired = brakingSpeed;
        float delta = acceleration * (8.583f / .684f) * elapsed / 1000;
        if (rampedSpeed < desired) {
            rampedSpeed += delta;
            if (rampedSpeed > desired) rampedSpeed = desired;
        } else {
            rampedSpeed -= delta;
            if (rampedSpeed < desired) rampedSpeed = desired;
        }
        uint16_t limited = uint16_t(rampedSpeed + .5f);
        if (limited < 1) limited = 1;
        if (limited != appliedSpeed) {
            io.write(selected, 32, limited, 2);
            appliedSpeed = limited;
        }
        if ((goal == maximum && position >= int(maximum - tolerance)) ||
            (goal == minimum && position <= int(minimum + tolerance))) {
            nextMaximum = goal == minimum;
            goal = position;
            io.write(selected, 30, goal, 2);
            io.write(selected, 32, 1, 2);
            appliedSpeed = 1;
            rampedSpeed = 1;
            settling = true;
            settledAt = cornerAt = now;
            movementAt = now;
            lastPosition = position;
        }
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
        if (cw < 0 || ccw <= cw || ccw > 4095) return false;
        int position = io.read(selected, 36, 2);
        if (position < cw || position > ccw) return false;
        // Re-arming never replays the goal left behind by a previous session.
        io.write(selected, 30, position, 2);
        io.write(selected, 32, 1, 2);
        heartbeat = now;
        permitted = true;
        fault = 0;
        return true;
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
        appliedSpeed = 1;
        rampedSpeed = 1;
        settling = false;
        starting = true;
        startupStage = 0;
        startupAt = now;
        sweeping = true;
        movementAt = lastPoll = now;
        lastPosition = position;
        return true;
    }

    bool configure(uint8_t address, uint16_t value, uint8_t size) {
        if (sweeping) {
            if (address == 84 && size == 2) return value == minimum;
            if (address == 86 && size == 2) return value == maximum;
            if (address == 91 && size == 2) return value == tolerance;
            if (address != 88 && address != 90) return false;
        }
        switch (address) {
        case 81:
            if (permitted || size != 1 || value > 252) return false;
            selected = value;
            minimum = maximum = 0;
            return true;
        case 84: if (size != 2 || value > 4095) return false; minimum = value; return true;
        case 86: if (size != 2 || value > 4095) return false; maximum = value; return true;
        case 88: if (size != 2 || value < 1 || value > 1023) return false; speed = value; return true;
        case 90:
            if (size != 1 || value < 1 || value > 254) return false;
            if (sweeping && acceleration != value) io.write(selected, 73, value, 1);
            acceleration = value;
            return true;
        case 91: if (size != 2 || value < 1 || value > 1023) return false; tolerance = value; return true;
        default: return false;
        }
    }

    bool writeServo(uint8_t id, uint8_t address, uint16_t value, uint8_t size, uint32_t now) {
        tick(now);
        if (address == 24 && size == 1 && value == 0) { stop(); return true; }
        if (!permitted || sweeping || id != selected) return false;
        bool valid = false;
        if (address == 24) valid = size == 1 && value == 1 && io.read(id, 34, 2) > 0;
        if (address == 32) valid = size == 2 && value <= 1023;
        if (address == 73) valid = size == 1 && value <= 254;
        if (address == 30 && size == 2) {
            int cw = io.read(id, 6, 2), ccw = io.read(id, 8, 2);
            valid = cw >= 0 && ccw > cw && ccw <= 4095 && value <= 4095 && int(value) >= cw && int(value) <= ccw;
        }
        if (!valid) return false;
        io.write(id, address, value, size);
        return true;
    }
};
#endif
