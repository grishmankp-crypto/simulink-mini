import time
from playwright.sync_api import sync_playwright

FRONTEND_URL = "http://127.0.0.1:3000"

failures = []

def check(name, cond, detail=""):
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {name}" + (f" -- {detail}" if detail and not cond else ""))
    if not cond:
        failures.append(name)

with sync_playwright() as p:
    browser = p.chromium.launch()
    page = browser.new_page(viewport={"width": 1600, "height": 900})

    console_errors = []
    page.on("console", lambda msg: console_errors.append(msg.text) if msg.type == "error" else None)

    page.goto(FRONTEND_URL, wait_until="networkidle")
    page.wait_for_timeout(1000)

    check("Page title/header renders", page.locator("text=simulink-mini editor").count() > 0)

    # Wait for backend status badge to say "connected"
    page.wait_for_selector("text=backend: connected", timeout=10000)
    check("Backend status badge shows connected", True)

    page.screenshot(path="/tmp/screenshot_1_initial.png")

    # --- Load the PID+Plant example ---
    page.click("text=Load PID+Plant example")
    page.wait_for_timeout(500)
    check("Example loads PID node onto canvas", page.locator("text=PID").count() > 0)
    check("Example loads Integrator node onto canvas", page.locator("text=Integrator").count() > 0)
    page.screenshot(path="/tmp/screenshot_2_example_loaded.png")

    # --- Run simulation ---
    page.click("text=Run simulation")
    page.wait_for_timeout(1500)
    error_banner = page.locator(".bg-red-50")
    check("No error banner after Run simulation", error_banner.count() == 0,
          error_banner.inner_text() if error_banner.count() else "")
    # The chart renders an SVG from recharts once data arrives.
    check("Chart SVG rendered after simulation", page.locator("svg.recharts-surface").count() > 0)
    page.screenshot(path="/tmp/screenshot_3_simulated.png")

    # --- Stream live ---
    page.click("text=Stream live")
    page.wait_for_timeout(3000)
    check("No error banner after Stream live", page.locator(".bg-red-50").count() == 0)
    page.screenshot(path="/tmp/screenshot_4_streamed.png")

    # --- Tune PID ---
    page.click("text=Tune PID")
    page.wait_for_timeout(300)
    check("Tune panel opens with PID block id prefilled",
          page.locator("input[value='pid1']").count() > 0)

    page.click("text=Run PSO auto-tune")
    # PSO run + network round trip; give it real time.
    page.wait_for_selector("text=Improvement factor", timeout=20000)
    check("Tune panel shows improvement factor", True)
    check("No error banner after PID tuning", page.locator(".bg-red-50").count() == 0)

    improvement_text = page.locator("text=Improvement factor").locator("..").inner_text()
    print("       Tune panel result:", improvement_text.replace("\n", " | "))
    page.screenshot(path="/tmp/screenshot_5_tuned.png")

    check("No browser console errors", len(console_errors) == 0, str(console_errors[:5]))

    browser.close()

print()
if not failures:
    print("ALL BROWSER E2E TESTS PASSED")
else:
    print(f"{len(failures)} TEST(S) FAILED: {failures}")
    exit(1)
