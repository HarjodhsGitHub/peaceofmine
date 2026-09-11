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
                page.goto(f'http://127.0.0.1:{server.server_port}')
                page.wait_for_function("document.querySelector('#latitude').textContent === '0.0000000'")
                self.assertTrue(page.locator('#altitude').text_content().startswith('0'))
                self.assertIn('GSA', page.locator('#dop').locator('..').text_content())
                self.assertEqual(page.evaluate("charts.find(c => c.def[0] === 'altitude').instance.data.datasets[0].data.at(-1).y"), 0)
                page.screenshot(path='/tmp/gnss-desktop.png', full_page=True)
                page.set_viewport_size({'width': 390, 'height': 844})
                page.wait_for_timeout(300)
                self.assertTrue(page.evaluate('document.documentElement.scrollWidth <= innerWidth'))
                page.screenshot(path='/tmp/gnss-mobile.png', full_page=True)
                with state.lock:
                    state.field_updated['latitude'] -= 20
                page.wait_for_function("document.querySelector('#latitude').textContent === '--'")
                self.assertIn('No fresh position', page.locator('#latitude').locator('..').text_content())
                page.route('**/api/state', lambda route: route.fulfill(status=503, body='offline'))
                page.wait_for_function("document.querySelector('#connection').textContent.includes('Viewer offline')")
                self.assertEqual(page.locator('#altitude').text_content(), '--')
                page.unroute('**/api/state')
                process_sentence(state, sentence, str(sentence))
                page.wait_for_function("document.querySelector('#latitude').textContent === '0.0000000'")
                self.assertEqual(errors, [])
                browser.close()
        finally:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    unittest.main()
