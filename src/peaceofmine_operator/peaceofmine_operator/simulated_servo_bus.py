"""Deterministic MX-64 plant behind the production driver, with no device access."""
import time


class SimulatedServoBus:
    def __init__(self, arm_id=1, probe_id=2):
        self.values = {}
        self.positions = {arm_id: 1533.0, probe_id: -200.0}
        self.probe_id = probe_id
        self.last = time.monotonic()
        self.sweep = None
        self.external_load_ratio = .02
        for ident in self.positions:
            self.values[ident] = {0: 310, 2: 42, 6: 4095 if ident == probe_id else 0,
                8: 4095, 14: 1023, 22: 1, 24: 0, 30: int(self.positions[ident]),
                32: 20, 34: 1023, 36: int(self.positions[ident]), 38: 0, 40: 0,
                42: 120, 43: 25, 46: 0, 68: 2048, 70: 0, 73: 4}

    def advance(self):
        now = time.monotonic()
        dt, self.last = min(.1, now-self.last), now
        for ident, values in self.values.items():
            pos = self.positions[ident]
            target = values[30]
            if self.sweep and self.sweep[0] == ident:
                _, low, high = self.sweep
                if abs(target-pos) < 3:
                    target = low if target == high else high
                    values[30] = target
            step = values[32] * .684 * 4096 / 360 * dt if values[24] else 0
            next_pos = pos + max(-step, min(step, target-pos))
            # The probe's physical top is at encoder zero.
            contact = ident == self.probe_id and next_pos >= 0 and target > 0 and values[24]
            self.positions[ident] = min(0, next_pos) if ident == self.probe_id else next_pos
            values.update({36: round(self.positions[ident]) & 65535, 40: 512 if contact else round(self.external_load_ratio*1023) if ident==self.probe_id else 20,
                           38: values[32] if abs(target-pos)>3 and values[24] else 0,
                           46: int(abs(target-pos)>3 and bool(values[24]))})

    def read(self, ident, address, size=2):
        self.advance()
        return 2 if ident == 253 and address == 80 else self.values[ident].get(address, 0)

    def read_block(self, ident, address, length):
        self.advance()
        result = bytearray(length)
        for register, value in self.values[ident].items():
            size = 1 if register in (2, 22, 24, 42, 43, 46, 70, 73) else 2
            if address <= register and register+size <= address+length:
                result[register-address:register-address+size] = (value & ((1 << (8*size))-1)).to_bytes(size, 'little')
        return bytes(result)

    def write(self, ident, address, value, size=2):
        self.advance()
        if address == 30 and ident == self.probe_id and value >= 32768:
            value -= 65536
        self.values[ident][address] = value
        if address == 24 and value == 0:
            self.sweep = None

    def prepare_motion(self, ident, allowed):
        return allowed()

    def run_sweep(self, ident, low, high, speed, acceleration, tolerance, allowed):
        if not allowed():
            return False
        if self.sweep != (ident, low, high):
            self.values[ident][30] = high
        self.sweep = ident, low, high
        self.values[ident].update({24: 1, 32: speed, 73: acceleration})
        return True

    def enable_multiturn(self, ident):
        self.values[ident].update({6: 4095, 8: 4095, 22: 1})

    def close(self):
        for values in self.values.values():
            values[24] = 0
