#!/usr/bin/env python3
"""Bounded MX-64 step-response tuning. Hardware requires --run; no EEPROM writes.

Uses the operator's RC interlock, exclusive USB access and firmware watchdog.
Results and raw samples are flushed to disk throughout the experiment. Selected
PID gains are volatile RAM settings; the JSON calibration report is not loaded
by the operator automatically. Never raises speed, acceleration or torque limits.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/peaceofmine_operator'))


class Abort(RuntimeError):
    pass


class Limits:
    def __init__(self, path):
        self.path = Path(path)
        self.raw = self.path.read_bytes()
        self.digest = hashlib.sha256(self.raw).hexdigest()
        config = json.loads(self.raw)
        arm = config['arm']
        self.ident, self.low, self.center, self.high = [arm[k] for k in
                                                     ('servo_id', 'minimum', 'center', 'maximum')]
        if (any(type(v) is not int for v in (self.ident, self.low, self.center, self.high)) or
                not 0 <= self.ident <= 252 or not 0 <= self.low < self.center < self.high <= 4095):
            raise ValueError('Invalid arm ID / calibrated joint limits')
        self.radius = min(228, (self.center-self.low)//3, (self.high-self.center)//3)
        if self.radius < 80:
            raise ValueError('Insufficient calibrated space for a guarded tuning test')
        motion = config.get('arm_motion', {})
        max_speed = motion.get('max_speed_deg_s', 116*.684)
        acceleration = motion.get('acceleration_deg_s2', 4*8.583)
        if any(type(v) not in (float, int) or not math.isfinite(v) or v <= 0
               for v in (max_speed, acceleration)):
            raise ValueError('Invalid motion limits')
        self.speed_limit = max(1, min(1023, math.floor(max_speed/.684+1e-9)))
        self.speed = min(58, self.speed_limit)
        self.acceleration = max(1, min(254, math.floor(acceleration/8.583+1e-9)))

    def unchanged(self):
        if self.path.read_bytes() != self.raw:
            raise Abort('operator_config.json changed during calibration')

    def target(self, value):
        if not self.low + 40 <= value <= self.high - 40:
            raise Abort(f'Target {value} lacks margin inside saved limits')
        return value


def metrics(rows):
    """Score settling and travel separately; goals are fixed during each step."""
    tail = [r for r in rows if r['elapsed'] >= rows[-1]['elapsed']-.8]
    dt = [(b['elapsed']-a['elapsed']) for a, b in zip(rows, rows[1:])]
    iae = sum(abs(b['target']-b['position'])*t for b, t in zip(rows[1:], dt))
    direction = 1 if rows[0]['target'] >= rows[0]['position'] else -1
    overshoot = max(0, max((r['position']-r['target'])*direction for r in rows))
    error = statistics.mean(abs(r['target']-r['position']) for r in tail)
    noise = statistics.pstdev(r['position'] for r in tail)
    current = max(abs(r['current']) for r in rows)
    return dict(score=iae+20*error+30*noise+10*overshoot, iae=iae,
                tail_error=error, tail_noise=noise, overshoot=overshoot, peak_current=current)


def improved(candidate, baseline):
    return (candidate['score'] < baseline['score']*.9 and
            candidate['overshoot'] <= max(8, baseline['overshoot']+3) and
            candidate['tail_noise'] <= max(3, baseline['tail_noise']+1) and
            candidate['peak_current'] <= max(.6, baseline['peak_current']*1.4))


def choose_candidate(candidates, baseline):
    # A lower scalar score must never displace a stable candidate if it fails
    # an independent oscillation, current or overshoot acceptance criterion.
    eligible = [candidate for candidate in candidates if improved(candidate, baseline)]
    return min(eligible, key=lambda r: r['score']) if eligible else baseline


class RosGate:
    def __init__(self, namespace):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from mavros_msgs.msg import State, RCIn
        from peaceofmine_operator.rc_safety import RcSafety
        self.ros = rclpy
        rclpy.init()
        self.node = rclpy.create_node('servo_pid_calibration')
        self.safety = RcSafety()
        self.node.create_subscription(State, f'/{namespace}/mavros/state', self.safety.update_state, 10)
        self.node.create_subscription(RCIn, f'/{namespace}/mavros/rc/in', self.safety.update_rc, qos_profile_sensor_data)

    def status(self):
        for _ in range(8):
            self.ros.spin_once(self.node, timeout_sec=0)
        return self.safety.servo_snapshot()

    def check(self):
        status = self.status()
        if not status['allowed']:
            raise Abort(status['reason'])

    def close(self):
        self.node.destroy_node()
        self.ros.shutdown()


class Experiment:
    def __init__(self, bus, limits, gate, output, clock=time.monotonic, sleep=time.sleep):
        self.bus, self.limits, self.gate = bus, limits, gate
        self.clock, self.sleep = clock, sleep
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.file = (self.output/'samples.csv').open('w', newline='')
        self.csv = csv.DictWriter(self.file, fieldnames=['trial', 'p', 'i', 'd', 'target', 'elapsed',
            'position', 'velocity', 'current', 'load', 'voltage', 'temperature', 'torque', 'goal', 'speed_limit'])
        self.csv.writeheader()
        self.original = None
        self.active = False
        self.report = dict(config_sha256=limits.digest, limits=[limits.low, limits.high],
                           trials=[], complete=False, restoration='not attempted')

    def save(self):
        temp = self.output/'report.tmp'
        temp.write_text(json.dumps(self.report, indent=2)+'\n')
        temp.replace(self.output/'report.json')

    def write(self, address, value, size=1, gateway=False):
        self.bus.write(253 if gateway else self.limits.ident, address, value, size)

    def stop(self):
        self.write(82, 0, gateway=True)
        self.active = False
        if self.bus.read(self.limits.ident, 24, 1) != 0:
            raise Abort('Torque-off readback failed')

    def gains(self, gains):
        self.stop()
        self.write(81, self.limits.ident, gateway=True)
        for address, value in zip((28, 27, 26), gains):
            self.write(address, value)
            if self.bus.read(self.limits.ident, address, 1) != value:
                raise Abort('PID readback failed')
        self.pid = gains

    def check(self):
        self.limits.unchanged()
        self.gate.check()

    def arm(self):
        self.check()
        self.write(84, self.limits.low, 2, True)
        self.write(86, self.limits.high, 2, True)
        self.write(95, 1, gateway=True)
        self.write(82, 1, gateway=True)
        self.active = True
        self.write(32, self.speed, 2)
        self.write(73, self.acceleration)
        self.check()
        self.write(24, 1)
        self.sleep(.03)
        if self.bus.read(self.limits.ident, 24, 1) != 1:
            raise Abort('Torque-enable readback failed')

    def sample(self, target, elapsed, trial):
        data = self.bus.read_block(self.limits.ident, 24, 20)
        word = lambda offset: int.from_bytes(data[offset:offset+2], 'little')
        velocity, load = word(14), word(16)
        current = (self.bus.read(self.limits.ident, 68)-2048)*.0045
        row = dict(trial=trial, p=self.pid[0], i=self.pid[1], d=self.pid[2], target=target,
                   elapsed=elapsed, position=word(12), torque=data[0], goal=word(6), speed_limit=word(8),
                   velocity=(velocity & 1023)*(-1 if velocity & 1024 else 1)*.684,
                   load=(load & 1023)/1023, voltage=data[18]/10, temperature=data[19], current=current)
        self.csv.writerow(row)
        self.file.flush()
        if not self.limits.low+10 <= row['position'] <= self.limits.high-10:
            raise Abort('Position reached outer guard margin')
        if (row['torque'] != 1 or row['temperature'] >= 60 or abs(current) >= 1.5 or
                row['load'] >= .7 or not 10 <= row['voltage'] <= 14):
            raise Abort(f'Health limit reached: {row}')
        return row

    def move(self, target, trial, duration):
        self.check()
        self.limits.target(target)
        self.write(82, 2, gateway=True)
        self.write(30, target, 2)
        start = self.clock()
        rows = []
        while self.clock()-start < duration:
            self.check()
            self.write(82, 2, gateway=True)
            rows.append(self.sample(target, self.clock()-start, trial))
            if len(rows) > 2 and self.clock()-start > 1 and abs(rows[-1]['position']-rows[0]['position']) < 3 and abs(target-rows[-1]['position']) > 80:
                raise Abort('No encoder progress under commanded motion')
            self.sleep(.02)
        if abs(rows[-1]['position']-target) > 80:
            raise Abort('Step failed to reach target')
        return metrics(rows)

    def trial(self, gains, name, radius):
        self.check()
        self.gains(gains)
        self.arm()
        # Identical low-to-high and high-to-low steps, with positioning unscored.
        low, high = self.limits.center-radius, self.limits.center+radius
        position = self.bus.read(self.limits.ident, 36)
        # Initial setup can start near an outer endpoint. Avoid one large
        # position-error step at full test speed before the scored movements.
        transit_speed = min(29, self.speed)
        self.write(32, transit_speed, 2)
        for _ in range(64):
            position = self.bus.read(self.limits.ident, 36)
            if abs(position-low) <= 128:
                break
            target = position + (128 if low > position else -128)
            self.move(target, name+'-positioning', 128*360/4096/(transit_speed*.684)+2)
        else:
            raise Abort('Positioning failed to converge')
        self.move(low, name+'-positioning', abs(position-low)*360/4096/(transit_speed*.684)+2)
        self.write(32, self.speed, 2)
        duration = 2*radius*360/4096/(self.speed*.684)+3
        parts = [self.move(target, name, duration) for target in (high, low)]
        self.stop()
        result = {key: (max(p[key] for p in parts) if key in ('overshoot', 'peak_current', 'tail_noise')
                        else statistics.mean(p[key] for p in parts)) for key in parts[0]}
        result.update(gains=list(gains), name=name, radius=radius)
        self.report['trials'].append(result)
        self.save()
        print(json.dumps(result), flush=True)
        return result

    def verify_sweep(self, wide=False, speeds=None, reversals=6):
        """Measure existing gains with the deployed sweep controller; no gain search."""
        saved_speed = saved_accel = None
        try:
            self.check()
            self.stop()
            self.write(81, self.limits.ident, gateway=True)
            self.pid = tuple(self.bus.read(self.limits.ident, a, 1) for a in (28, 27, 26))
            self.report['gains'] = self.pid
            saved_speed = self.bus.read(self.limits.ident, 32)
            saved_accel = self.bus.read(self.limits.ident, 73, 1)
            radius = self.limits.radius*2
            low, high = self.limits.center-radius, self.limits.center+radius
            if wide:
                low, high = self.limits.low+80, self.limits.high-80
                radius = (high-low)/2
            self.limits.target(low)
            self.limits.target(high)
            if not low <= self.bus.read(self.limits.ident, 36) <= high:
                raise Abort('Position must be inside the sweep test window')
            speeds = speeds or (min(29, self.limits.speed), self.limits.speed, min(116, self.limits.speed_limit))
            if (type(reversals) is not int or not 3 <= reversals <= 12 or
                    any(type(v) is not int or not 1 <= v <= min(116, self.limits.speed_limit) for v in speeds)):
                raise ValueError('Invalid sweep verification limits')
            for speed in sorted(set(speeds)):
                self.check()
                self.write(84, low, 2, True)
                self.write(86, high, 2, True)
                self.write(88, speed, 2, True)
                self.write(90, min(self.limits.acceleration, saved_accel or self.limits.acceleration), 1, True)
                self.write(91, 23, 2, True)
                self.write(82, 1, gateway=True)
                self.write(94, 1, gateway=True)
                start = self.clock()
                while self.bus.read(self.limits.ident, 24, 1) != 1:
                    self.check()
                    self.write(82, 2, gateway=True)
                    if self.clock()-start > .3:
                        raise Abort('Sweep startup failed')
                    self.sleep(.02)
                previous = None
                changes = 0
                rows = []
                deadline = 8*(2*radius*360/4096/(speed*.684)+3)
                while changes < reversals:
                    self.check()
                    self.write(82, 2, gateway=True)
                    elapsed = self.clock()-start
                    if elapsed > deadline:
                        raise Abort('Sweep failed to complete requested reversals')
                    row = self.sample(0, elapsed, f'sweep-{speed}')
                    if row['goal'] in (low, high):
                        if previous is not None and row['goal'] != previous:
                            changes += 1
                        previous = row['goal']
                    rows.append(row)
                    self.sleep(.02)
                self.stop()
                result = dict(speed=speed, reversals=changes, duration=self.clock()-start,
                              peak_current=max(abs(r['current']) for r in rows),
                              position_range=[min(r['position'] for r in rows), max(r['position'] for r in rows)])
                self.report['trials'].append(result)
                self.save()
                print(json.dumps(result), flush=True)
            self.report['complete'] = True
        except BaseException as exc:
            self.report['error'] = str(exc)
            raise
        finally:
            try:
                self.stop()
                if saved_speed is not None:
                    self.check()
                    self.write(82, 1, gateway=True)
                    self.write(32, saved_speed, 2)
                    self.write(73, saved_accel)
                    self.stop()
                    if (self.bus.read(self.limits.ident, 32) != saved_speed or
                            self.bus.read(self.limits.ident, 73, 1) != saved_accel):
                        raise Abort('Motion-register restoration readback failed')
                self.report['restoration'] = 'torque off; PID unchanged; motion registers restored'
            except BaseException as exc:
                self.report['restoration'] = f'incomplete: {exc}'
                try:
                    self.stop()
                except Exception:
                    pass
            finally:
                self.save()
                self.file.close()

    def run(self, keep_best=False):
        selected = None
        try:
            if self.bus.read(253, 79, 1) != 1:
                raise Abort('Install firmware with stopped PID writes and bounded test mode')
            self.stop()
            self.write(81, self.limits.ident, gateway=True)
            if self.bus.read(self.limits.ident, 0) != 310:
                raise Abort('Only MX-64 model 310 is supported')
            if self.bus.read(self.limits.ident, 6) >= self.bus.read(self.limits.ident, 8):
                raise Abort('Arm must be in calibrated joint mode')
            self.original = tuple(self.bus.read(self.limits.ident, a, 1) for a in (28, 27, 26))
            self.old_speed = self.bus.read(self.limits.ident, 32)
            self.old_accel = self.bus.read(self.limits.ident, 73, 1)
            self.speed = min(self.limits.speed, self.old_speed or self.limits.speed)
            self.acceleration = min(self.limits.acceleration, self.old_accel or self.limits.acceleration)
            self.report.update(original_gains=self.original, speed=self.speed, acceleration=self.acceleration)
            self.save()
            radius = self.limits.radius//2
            baseline = self.trial(self.original, 'baseline', radius)
            candidates = [baseline]
            # Keep I zero initially: integral wind-up can amplify endpoint hunting.
            for p, d in ((40, 0), (48, 0), (64, 0), (48, 8), (64, 8)):
                candidates.append(self.trial((p, 0, d), f'p{p}-d{d}', radius))
            winner = choose_candidate(candidates, baseline)
            if winner['tail_error'] > 3 and winner['tail_noise'] < 2:
                integral = self.trial((winner['gains'][0], 1, winner['gains'][2]), 'integral-1', radius)
                if improved(integral, winner):
                    winner = integral
            selected = self.original
            if improved(winner, baseline):
                # Repeat baseline and candidate on larger held-out movements.
                reference = self.trial(self.original, 'validation-baseline', self.limits.radius)
                validation = self.trial(winner['gains'], 'validation-candidate', self.limits.radius)
                repeat = self.trial(winner['gains'], 'validation-repeat', self.limits.radius)
                if improved(validation, reference) and improved(repeat, reference):
                    selected = tuple(winner['gains'])
            self.report.update(complete=True, selected_gains=selected, keep_best=keep_best)
        except BaseException as exc:
            self.report['error'] = str(exc)
            raise
        finally:
            try:
                self.stop()
                if self.original is not None:
                    self.gains(selected if self.report['complete'] and keep_best else self.original)
                    # Motion limits are restored with torque off, a short lease,
                    # and the same bounds; torque is never re-enabled here.
                    self.check()
                    self.write(84, self.limits.low, 2, True)
                    self.write(86, self.limits.high, 2, True)
                    self.write(95, 1, gateway=True)
                    self.write(82, 1, gateway=True)
                    self.write(32, self.old_speed, 2)
                    self.write(73, self.old_accel)
                    if (self.bus.read(self.limits.ident, 32) != self.old_speed or
                            self.bus.read(self.limits.ident, 73, 1) != self.old_accel):
                        raise Abort('Motion-register restoration readback failed')
                    self.stop()
                    self.report['restoration'] = 'gains and motion registers verified; torque off'
            except BaseException as exc:
                self.report['restoration'] = f'incomplete: {exc}'
                try:
                    self.stop()
                except Exception:
                    pass
            self.save()
            self.file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'src/peaceofmine_operator/operator_config.json')
    parser.add_argument('--device', default='/dev/ttyUSB0')
    parser.add_argument('--namespace', default='self')
    parser.add_argument('--output', type=Path, default=ROOT/'tools/results'/time.strftime('%Y%m%d-%H%M%S'))
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--keep-best', action='store_true')
    parser.add_argument('--verify-sweep', action='store_true', help='Measure six reversals at three speeds using current gains')
    parser.add_argument('--wide-sweep', action='store_true', help='Use saved position limits with 80-tick interior margins')
    args = parser.parse_args()
    limits = Limits(args.config)
    if not args.run:
        print(json.dumps(dict(id=limits.ident, limits=[limits.low, limits.high], center=limits.center,
                             validation_radius=limits.radius, hardware_started=False), indent=2))
        return
    from peaceofmine_operator.servo_transport import ArbotiX
    gate = RosGate(args.namespace)
    bus = None
    try:
        deadline = time.monotonic()+12
        while not gate.status()['allowed'] and time.monotonic() < deadline:
            time.sleep(.02)
        gate.check()
        bus = ArbotiX(args.device)
        experiment = Experiment(bus, limits, gate, args.output)
        if args.verify_sweep:
            experiment.verify_sweep(args.wide_sweep)
        else:
            experiment.run(args.keep_best)
    finally:
        if bus:
            bus.close()
        gate.close()


if __name__ == '__main__':
    main()
