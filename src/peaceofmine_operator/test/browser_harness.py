"""Bounded fixture rendering, including recent Chromium builds."""
import subprocess


def render(page, directory, width=1440):
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        result=subprocess.run(['chromium','--headless','--no-sandbox','--disable-gpu',
            f'--window-size={width},1000','--timeout=10000','--virtual-time-budget=2000',
            '--user-data-dir='+str(directory)+'/profile','--dump-dom',page.as_uri()],
            capture_output=True,text=True,timeout=30)
        return result.stdout+result.stderr[-2000:]
    import shutil
    with sync_playwright() as pw:
        browser=pw.chromium.launch(executable_path=shutil.which('chromium'),args=['--no-sandbox'])
        tab=browser.new_page(viewport={'width':width,'height':1000})
        tab.goto(page.as_uri(),wait_until='domcontentloaded')
        tab.wait_for_function('document.body.childElementCount === 0',timeout=15000)
        result=tab.locator('body').evaluate('(element) => element.outerHTML')
        browser.close()
        return result
