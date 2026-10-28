"""Browser integration against a synthetic ADC; no I2C access."""
import json
import os
import threading
import unittest
from http.server import ThreadingHTTPServer

from playwright.sync_api import sync_playwright
from server import Acquisition, Handler


class BrowserTests(unittest.TestCase):
    def test_acquisition_configuration_and_layout(self):
        acquisition = Acquisition(demo=os.environ.get('ADS1115_HARDWARE_TEST') != '1')
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        server.acquisition = acquisition
        acquisition.thread.start()
        serving = threading.Thread(target=server.serve_forever, daemon=True)
        serving.start()
        try:
            with sync_playwright() as pw:
                browser = pw.chromium.launch(executable_path='/usr/bin/chromium', args=['--no-sandbox'])
                page = browser.new_page(viewport={'width': 1440, 'height': 1000})
                errors = []
                page.on('pageerror', lambda e: errors.append(str(e)))
                page.goto(f'http://127.0.0.1:{server.server_port}')
                page.wait_for_function("document.querySelector('#status').textContent === 'Ready'")
                self.assertEqual(page.locator('.channel').count(), 4)
                page.locator('#run').click()
                page.wait_for_function('history.length > 40')
                self.assertEqual(page.locator('.flag', has_text='Live').count(), 4)
                self.assertTrue(page.evaluate("[...charts.values()].every(c=>c.data.datasets[0].data.length>3)"))
                page.wait_for_timeout(1200)
                self.assertGreaterEqual(page.evaluate(r"Number(document.querySelector('#refresh').textContent.match(/\d+/)[0])"), 8)
                # Dense traces stay bounded without losing extrema or time order.
                self.assertTrue(page.evaluate("""() => {
                    const samples = Array.from({length:30000}, (_,i) =>
                        ({time:i/1000, volts:i===12345 ? 50 : i===12346 ? -50 : Math.sin(i)}));
                    const points = plotPoints(samples, 30, 30, 'volts', 600);
                    return points.length <= 1200 && points[0].x === -30 &&
                        points.at(-1).x === samples.at(-1).time-30 &&
                        Math.max(...points.map(p=>p.y)) === 50 &&
                        Math.min(...points.map(p=>p.y)) === -50 &&
                        points.every((p,i)=>i===0 || p.x > points[i-1].x);
                }"""))
                # Rendering keeps scrolling between network batches.
                page.route('**/api/state*', lambda route: route.abort())
                self.assertTrue(page.evaluate("""async () => {
                    const original = render;
                    let count = 0;
                    render = now => { count++; original(now); };
                    // Allow the last in-flight request to settle, then simulate a
                    // connected source with no incoming batches for 200 ms.
                    await new Promise(resolve=>setTimeout(resolve, 100));
                    state.connected = true; pendingFrame = false; count = 0;
                    await new Promise(resolve=>setTimeout(resolve, 200));
                    render = original; state.connected = false;
                    return count > 2;
                }"""))
                page.unroute('**/api/state*')
                page.wait_for_function('state.connected')
                page.screenshot(path='/tmp/ads1115-desktop.png', full_page=True)
                page.set_viewport_size({'width': 390, 'height': 844})
                page.wait_for_timeout(300)
                self.assertTrue(page.evaluate('document.documentElement.scrollWidth <= innerWidth'))
                page.screenshot(path='/tmp/ads1115-mobile.png', full_page=True)
                page.locator('[data-view=settings]').click()
                self.assertTrue(page.evaluate('document.documentElement.scrollWidth <= innerWidth'))
                self.assertIn('lower rates reduce noise', page.locator('[name=rate]').locator('..').inner_text())
                page.locator('[name=mode]').select_option('continuous')
                page.get_by_role('button', name='Apply settings', exact=True).click()
                page.wait_for_function("document.querySelector('#error').textContent.includes('exactly one')")
                for i in (5, 6, 7):
                    page.locator(f'[name=enabled{i}]').uncheck()
                page.locator('[name=multiplier4]').fill('2')
                page.locator('[name=offset4]').fill('-0.5')
                page.locator('[name=rate]').select_option('250')
                page.get_by_role('button', name='Apply settings', exact=True).click()
                page.wait_for_function('state.config.mode === "continuous" && state.applied === state.revision')
                self.assertTrue(page.locator('#error').is_hidden())
                page.locator('[data-view=monitor]').click()
                page.wait_for_function('history.length > 8')
                self.assertEqual(page.locator('.channel').count(), 1)
                self.assertTrue(page.evaluate('history.every(s=>Math.abs(s.scaled-(2*s.volts-.5))<1e-8)'))
                with page.expect_download() as download:
                    page.locator('#export').click()
                self.assertEqual(download.value.suggested_filename, 'ads1115.csv')
                page.locator('#run').click()
                page.wait_for_function('!state.running')
                seq = acquisition.snapshot()['sequence']
                page.wait_for_timeout(350)
                self.assertEqual(seq, acquisition.snapshot()['sequence'])
                page.locator('[data-view=settings]').click()
                page.locator('[name=alert]').select_option('ready')
                page.get_by_role('button', name='Apply settings', exact=True).click()
                page.wait_for_function('state.config.alert === "ready" && state.applied === state.revision')
                page.locator('[data-view=registers]').click()
                page.wait_for_function("document.querySelector('#register-values').textContent.includes('0x8000')")
                self.assertTrue(page.evaluate('document.documentElement.scrollWidth <= innerWidth'))
                page.route('**/api/state*', lambda route: route.fulfill(status=503, body='offline'))
                page.wait_for_function("document.querySelector('#status').textContent === 'Server offline'")
                page.unroute('**/api/state*')
                page.wait_for_function("document.querySelector('#status').textContent === 'Ready'")
                response = page.request.post(f'http://127.0.0.1:{server.server_port}/api/run',
                                             data=json.dumps({'running': True}),
                                             headers={'Content-Type': 'application/json', 'Origin': 'http://evil.invalid'})
                self.assertEqual(response.status, 403)
                self.assertEqual(errors, [])
                browser.close()
        finally:
            server.shutdown()
            server.server_close()
            serving.join()
            acquisition.close()


if __name__ == '__main__':
    unittest.main()
