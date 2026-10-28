import csv
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from tune_pid import Abort, Experiment, Limits, improved, choose_candidate
from compare_sweep_pid import jitter


class SimBus:
    def __init__(self):
        self.reg = {0:310, 6:0, 8:4095, 24:0, 26:0, 27:0, 28:32, 30:1514,
                    32:116, 36:1514, 68:2048, 73:4}
        self.now = 0
        self.permitted = False
        self.last = 0
        self.position = 1514.
        self.targets = []
        self.fail = False
        self.sweeping = False
        self.bounds = {}

    def write(self, ident, address, value, size):
        if ident == 253:
            if address in (84, 86):
                self.bounds[address] = value
            if address == 88:
                self.reg[32] = value
            if address == 94 and value == 1:
                self.sweeping = True
                self.reg[24] = 1
                self.reg[30] = self.bounds[86]
            if address == 82:
                self.permitted = value != 0
                self.last = self.now
                if value == 0:
                    self.reg[24] = 0
                    self.sweeping = False
                if value == 1:
                    self.reg[30] = int(self.position)
            return
        if address in (26, 27, 28):
            assert not self.reg[24] and not self.permitted
        if address == 30:
            assert 551 <= value <= 2515
            self.targets.append(value)
        if address == 24 and value == 1:
            assert self.permitted
        self.reg[address] = value

    def read(self, ident, address, size=2):
        if ident == 253:
            return 1
        return self.reg[address]

    def read_block(self, ident, address, length):
        if self.fail and self.now > 1:
            raise Abort('simulated unplug')
        data = bytearray(20)
        data[0] = self.reg[24]
        data[6:8] = self.reg[30].to_bytes(2, 'little')
        data[8:10] = self.reg[32].to_bytes(2, 'little')
        data[12:14] = int(self.position).to_bytes(2, 'little')
        data[18:20] = bytes([120, 35])
        return data

    def sleep(self, dt):
        self.now += dt
        if self.now-self.last >= .350:
            self.reg[24] = 0
        if self.reg[24]:
            distance = self.reg[30]-self.position
            travel = self.reg[32]*.684*4096/360*dt
            self.position += max(-travel, min(travel, distance))
            self.reg[36] = int(self.position)
            if self.sweeping and abs(self.position-self.reg[30]) < 3:
                self.reg[30] = self.bounds[84] if self.reg[30] == self.bounds[86] else self.bounds[86]


class Gate:
    def check(self):
        pass


class TuningTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.root/'config.json'
        self.config.write_text(json.dumps({'arm':dict(servo_id=1, minimum=511, center=1514, maximum=2555)}))
        self.limits = Limits(self.config)

    def experiment(self, bus, gate=None):
        return Experiment(bus, self.limits, gate or Gate(), self.root/'results',
                          clock=lambda:bus.now, sleep=bus.sleep)

    def test_whole_sequence_bounds_and_original_restore(self):
        bus = SimBus()
        experiment = self.experiment(bus)
        with contextlib.redirect_stdout(io.StringIO()):
            experiment.run(keep_best=True)
        self.assertTrue(experiment.report['complete'])
        self.assertEqual(experiment.report['selected_gains'], (32, 0, 0))
        self.assertGreater(len(bus.targets), 15)
        self.assertEqual([bus.reg[a] for a in (28,27,26,32,73,24)], [32,0,0,116,4,0])

    def test_sweep_verification_stops_after_reversals(self):
        bus = SimBus()
        experiment = self.experiment(bus)
        with contextlib.redirect_stdout(io.StringIO()):
            experiment.verify_sweep()
        self.assertTrue(experiment.report['complete'])
        self.assertEqual([r['reversals'] for r in experiment.report['trials']], [6, 6, 6])
        self.assertEqual(bus.reg[24], 0)
        self.assertEqual(bus.reg[28], 32)

    def test_communication_failure_stops_and_restores(self):
        bus = SimBus()
        bus.fail = True
        experiment = self.experiment(bus)
        with self.assertRaisesRegex(Abort, 'unplug'):
            experiment.run(True)
        self.assertFalse(experiment.report['complete'])
        self.assertEqual([bus.reg[a] for a in (28,27,26,24)], [32,0,0,0])

    def test_rc_loss_does_not_rearm(self):
        bus = SimBus()
        class Kill:
            def check(self):
                if bus.now > .5:
                    raise Abort('RC kill')
        experiment = self.experiment(bus, Kill())
        with self.assertRaisesRegex(Abort, 'RC kill'):
            experiment.run(True)
        self.assertEqual(bus.reg[24], 0)
        self.assertFalse(bus.permitted)
        self.assertEqual(bus.reg[28], 32)

    def test_jitter_audit_distinguishes_ripple_from_constant_speed(self):
        file = self.root/'samples.csv'
        def record(oscillates):
            with file.open('w') as stream:
                writer = csv.DictWriter(stream, fieldnames=['speed_limit','goal','position','velocity','elapsed'])
                writer.writeheader()
                for k in range(150):
                    writer.writerow(dict(speed_limit=29,goal=2475,position=600+8*k,
                                         velocity=20+((-12 if k%2 else 12) if oscillates else 0),elapsed=k*.04))
            return jitter(file)
        steady, ripple = record(False), record(True)
        self.assertEqual(steady['raw_speed_sd'], 0)
        self.assertGreater(ripple['raw_speed_sd'], 11)
        self.assertLess(steady['encoder_speed_sd'], 1e-10)

    def test_config_change_and_target_guard(self):
        with self.assertRaises(Abort):
            self.limits.target(530)
        self.config.write_text('{}')
        with self.assertRaises(Abort):
            self.limits.unchanged()

    def test_selection_rejects_fast_but_oscillatory_result(self):
        base = dict(score=100, overshoot=2, tail_noise=1, peak_current=.2)
        self.assertFalse(improved(dict(base, score=50, tail_noise=5), base))
        self.assertFalse(improved(dict(base, score=50, peak_current=1), base))
        self.assertTrue(improved(dict(base, score=80), base))
        stable = dict(base, score=80)
        unstable = dict(base, score=50, tail_noise=5)
        self.assertEqual(choose_candidate([base, unstable, stable], base), stable)


if __name__ == '__main__':
    unittest.main()
