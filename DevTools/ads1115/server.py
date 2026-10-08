#!/usr/bin/env python3
"""Standalone ADS1115 bench service. No ROS or frontend build required."""
import argparse
import copy
import json
import math
import signal
import threading
import time
from collections import deque
from itertools import takewhile
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from adc import ADS1115, DEFAULTS, INPUTS, RANGES, config_word, validate

WEB = Path(__file__).parent / 'web'


class Acquisition:
    def __init__(self, demo=False, config=None):
        self.demo = demo
        self.config = validate(config or copy.deepcopy(DEFAULTS))
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.running = False
        self.revision = 1
        self.applied = 0
        self.connected = False
        self.error = None
        self.registers = {}
        self.samples = deque(maxlen=30000)
        self.sequence = 0
        self.device = None
        self.thread = threading.Thread(target=self.work, daemon=True)

    def update(self, value):
        config = validate(value)
        with self.lock:
            self.config = config
            self.revision += 1
            self.samples.clear()

    def run(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('running must be a boolean')
        with self.lock:
            self.running = enabled
            if not enabled and self.demo and 'config' in self.registers:
                self.registers['config'] |= 0x8103
            if not enabled and self.device:
                try:
                    self.device.idle()
                    self.registers = self.device.registers()
                except OSError as exc:
                    self.error = str(exc)
                    self.disconnect()
                    raise

    def snapshot(self, after=0):
        with self.lock:
            samples = list(takewhile(lambda s: s['seq'] > after, reversed(self.samples)))
            samples.reverse()
            dropped = max(0, (samples[0]['seq'] - after - 1)) if samples and after else 0
            return dict(demo=self.demo, config=copy.deepcopy(self.config),
                        revision=self.revision, applied=self.applied,
                        connected=self.connected, running=self.running, error=self.error,
                        registers=self.registers.copy(), samples=samples[:5000],
                        sequence=self.sequence, dropped=dropped, now=time.time(), inputs=INPUTS)

    def disconnect(self):
        if self.device:
            try:
                self.device.close()
            except OSError:
                pass
            self.device = None
        self.connected = False
        self.applied = 0

    def work(self):
        index = 0
        next_register_read = 0
        while not self.stop_event.is_set():
            delay = 0.05
            with self.lock:
                c = self.config
                try:
                    if self.applied != self.revision:
                        self.disconnect()
                        if not self.demo:
                            self.device = ADS1115(c['bus'], c['address'])
                            self.registers = self.device.configure(c)
                        else:
                            lo, hi = (0, 32768) if c['alert'] == 'ready' else (c['low'] & 65535, c['high'] & 65535)
                            mux = next((ch['mux'] for ch in c['channels'] if ch['enabled']), 4)
                            self.registers = dict(config=config_word(c, mux, idle=True) | 0x8000, low=lo, high=hi)
                        self.applied = self.revision
                        self.connected = True
                        self.error = None
                        index = 0
                        next_register_read = 0
                    channels = [ch for ch in c['channels'] if ch['enabled']]
                    if self.running and channels:
                        ch = channels[index % len(channels)]
                        mux = ch['mux']
                        if self.demo:
                            time.sleep(1 / c['rate'])
                            voltage = 0.12 * math.sin(time.monotonic() * (0.7 + mux * 0.13))
                            voltage += (mux - 3) * 0.35 if mux >= 4 else 0
                            raw = max(-32768, min(32767, round(voltage * 32768 / RANGES[c['pga']])))
                            self.registers['config'] = config_word(c, mux) | (0x8000 if c['mode'] == 'single' else 0)
                        else:
                            raw = self.device.sample(c, mux)
                            # Diagnostic reads must not consume three bus transfers per sample.
                            if time.monotonic() >= next_register_read:
                                self.registers = self.device.registers()
                                next_register_read = time.monotonic() + 1
                        self.registers['conversion'] = raw & 65535
                        volts = raw * RANGES[c['pga']] / 32768
                        self.sequence += 1
                        self.samples.append(dict(seq=self.sequence, time=time.time(), mux=mux,
                                                 raw=raw, volts=volts,
                                                 scaled=volts * ch['multiplier'] + ch['offset'],
                                                 clipped=raw in (-32768, 32767)))
                        index += 1
                        delay = c['interval_ms'] / 1000 if index % len(channels) == 0 else 0
                except (OSError, TimeoutError, ImportError) as exc:
                    self.error = f'{type(exc).__name__}: {exc}'
                    self.disconnect()
                    delay = 1
            self.stop_event.wait(max(0.0001, delay))
        with self.lock:
            self.disconnect()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=3)


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEB), **kwargs)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        super().end_headers()

    def reply(self, value, status=200):
        body = json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlsplit(self.path)
        if url.path == '/api/state':
            try:
                after = max(0, int(parse_qs(url.query).get('after', ['0'])[0]))
                self.reply(self.server.acquisition.snapshot(after))
            except ValueError as exc:
                self.reply(dict(error=str(exc)), 400)
        else:
            super().do_GET()

    def do_POST(self):
        # JSON-only, same-origin writes prevent cross-site form requests.
        origin = self.headers.get('Origin')
        if (origin and origin != 'http://' + self.headers.get('Host', '')) or self.headers.get('Sec-Fetch-Site') == 'cross-site':
            self.reply(dict(error='Cross-origin writes are disabled'), 403)
            return
        if self.headers.get('Content-Type') != 'application/json':
            self.reply(dict(error='Expected application/json'), 415)
            return
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 16384:
                raise ValueError('Invalid request size')
            value = json.loads(self.rfile.read(size))
            if self.path == '/api/settings':
                self.server.acquisition.update(value)
            elif self.path == '/api/run':
                if not isinstance(value, dict) or 'running' not in value:
                    raise ValueError('Missing running value')
                self.server.acquisition.run(value['running'])
            else:
                self.reply(dict(error='Not found'), 404)
                return
            self.reply(dict(ok=True))
        except (ValueError, TypeError) as exc:
            self.reply(dict(error=str(exc)), 400)
        except OSError as exc:
            self.reply(dict(error=str(exc)), 503)

    def log_message(self, *_):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8091)
    parser.add_argument('--bus', type=int, default=1)
    parser.add_argument('--address', type=lambda s: int(s, 0), default=0x48)
    parser.add_argument('--demo', action='store_true')
    args = parser.parse_args()
    config = copy.deepcopy(DEFAULTS)
    config.update(bus=args.bus, address=args.address)
    acquisition = Acquisition(args.demo, config)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.acquisition = acquisition
    acquisition.thread.start()
    def terminate(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    print(f'ADS1115 {"DEMO" if args.demo else "I2C"}: http://{args.host}:{args.port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        acquisition.close()


if __name__ == '__main__':
    main()
