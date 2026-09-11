"""Browser regression: run with a Python environment containing Playwright.

Uses system Chromium and a local HTTP server; no UART, NTRIP or internet needed.
"""
import threading
import unittest
from http.server import ThreadingHTTPServer

import pynmea2
try:
    from playwright.sync_api import sync_playwright
except ImportError:
    sync_playwright = None
from gnss_viewer import DashboardHandler, GnssState, process_sentence


@unittest.skipIf(sync_playwright is None, "Optional browser test requires playwright")
class DashboardTests(unittest.TestCase):
    def test_live_missing_stale_and_recovery(self):
        state = GnssState('test-uart', 115200)
        sentence = pynmea2.GGA('GN', 'GGA', (
            '120000', '0000.0000', 'N', '00000.0000', 'E', '4', '12',
            '0.8', '0.0', 'M', '0.0', 'M', '', ''))
        process_sentence(state, sentence, str(sentence))
        state.update(horizontal_accuracy_m=2.5, accuracy_source='UBX NAV-PVT')
        self.assertEqual(state.snapshot()['latitude'], 0)
        self.assertEqual(state.snapshot()['altitude_m'], 0)
        DashboardHandler.state = state
        server = ThreadingHTTPServer(('127.0.0.1', 0), DashboardHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            with sync_playwright() as pw:
                browser = pw.chromium.launch(executable_path='/usr/bin/chromium', args=['--no-sandbox'])
                page = browser.new_page(viewport={'width': 1440, 'height': 1000})
                errors = []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.route('https://**/*', lambda route: route.abort())
                process_sentence(state, sentence, str(sentence))
                state.update(horizontal_accuracy_m=2.5, accuracy_source='UBX NAV-PVT')
                page.goto(f'http://127.0.0.1:{server.server_port}')
                page.wait_for_function("document.querySelector('#latitude').textContent === '0.0000000'")
                self.assertTrue(page.locator('#altitude').text_content().startswith('0'))
                self.assertIn('GSA', page.locator('#dop').locator('..').text_content())
                self.assertEqual(page.evaluate("charts.find(c => c.def[0] === 'altitude').instance.data.datasets[0].data.at(-1).y"), 0)
                self.assertEqual(page.evaluate('accuracyCircle.getRadius()'), 2.5)
                self.assertEqual(page.evaluate('accuracyCircle.getLatLng().lat'), 0)
                page.screenshot(path='/tmp/gnss-desktop.png', full_page=True)
                page.set_viewport_size({'width': 390, 'height': 844})
                page.wait_for_timeout(300)
                self.assertTrue(page.evaluate('document.documentElement.scrollWidth <= innerWidth'))
                page.screenshot(path='/tmp/gnss-mobile.png', full_page=True)
                with state.lock:
                    state.field_updated['latitude'] -= 20
                page.wait_for_function("document.querySelector('#latitude').textContent === '--'")
                self.assertTrue(page.evaluate('accuracyCircle === null'))
                self.assertIn('No fresh position', page.locator('#latitude').locator('..').text_content())
                page.route('**/api/state', lambda route: route.fulfill(status=503, body='offline'))
                page.wait_for_function("document.querySelector('#connection').textContent.includes('Viewer offline')")
                self.assertEqual(page.locator('#altitude').text_content(), '--')
                page.unroute('**/api/state')
                process_sentence(state, sentence, str(sentence))
                page.wait_for_function("document.querySelector('#latitude').textContent === '0.0000000'")
                page.evaluate("""() => {
                    referencePoint = [59.350911, 18.067915];
                    const s = {...samples.at(-1).s, latitude: 59.3506605, longitude: 18.0684725,
                      horizontal_accuracy_m: 1.043, field_age_s: undefined, age_s: 0};
                    updateMap(s, performance.now());
                }""")
                self.assertIn('42.1 m', page.locator('#reference-status').inner_text())
                self.assertIn('OUTSIDE', page.locator('#reference-status').inner_text())
                self.assertEqual(page.evaluate('accuracyCircle.getRadius()'), 1.043)
                page.locator('#clear-reference').click()
                self.assertTrue(page.evaluate('referenceMarker === null && referenceLine === null'))
                for quality, title in [(1, 'Regular GPS'), (2, 'Differential GPS'), (4, 'RTK fixed'), (5, 'RTK float')]:
                    page.evaluate("""q => updateStatus({...samples.at(-1).s, connected: true,
                        age_s: 0, field_age_s: undefined, fix_quality: q,
                        ntrip_enabled: true, ntrip_status: 'Receiving RTCM corrections',
                        correction_age_s: 0.2, corrections_bytes: 2048})""", quality)
                    self.assertEqual(page.locator('#solution-title').inner_text(), title)
                    self.assertEqual(page.locator('#service-status').inner_text(), 'Connected')
                page.evaluate("updateStatus({...samples.at(-1).s, ntrip_enabled: true, ntrip_status: 'NTRIP error: disconnected', correction_age_s: 0.5})")
                self.assertEqual(page.locator('#service-status').inner_text(), 'Connection error')
                page.evaluate('updateStatus(null)')
                self.assertEqual(page.locator('#solution-title').inner_text(), 'Viewer offline')
                self.assertEqual(page.locator('#service-status').inner_text(), 'Unknown')
                self.assertEqual(errors, [])
                browser.close()
        finally:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    unittest.main()
