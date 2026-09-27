"""Render the promo (index.html) to video, frame by frame.

  python render.py stills 1.2 8.9 17.4        # QA stills -> out/stills/
  python render.py video [--fps 60] [--workers 8]

Needs: playwright (uses the system Google Chrome) and imageio-ffmpeg.
Every frame is painted by window.renderAt(t), so frames are independent
and can be captured in parallel.
"""
import argparse
import functools
import http.server
import os
import socketserver
import subprocess
import sys
import threading
from multiprocessing import Process
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"


def serve():
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(HERE))
    handler.func.log_message = lambda *a: None
    httpd = socketserver.ThreadingTCPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd, httpd.server_address[1]


def open_page(pw, port):
    browser = pw.chromium.launch(channel="chrome", args=["--force-color-profile=srgb", "--disable-lcd-text"])
    page = browser.new_page(viewport={"width": 1920, "height": 1080}, device_scale_factor=1)
    page.goto(f"http://127.0.0.1:{port}/index.html")
    page.wait_for_function("window.ready !== undefined")
    page.evaluate("window.ready")
    return browser, page


def worker(port, frames, fps, frame_dir):
    from playwright.sync_api import sync_playwright
    with sync_playwright() as pw:
        browser, page = open_page(pw, port)
        stage = page.locator("#stage")
        for f in frames:
            page.evaluate(f"renderAt({f / fps})")
            stage.screenshot(path=str(frame_dir / f"f{f:05d}.png"), animations="disabled")
        browser.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["stills", "video", "events"])
    ap.add_argument("times", nargs="*", type=float)
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--audio", default=str(OUT / "soundtrack.wav"))
    args = ap.parse_args()
    httpd, port = serve()

    if args.mode == "events":
        import json
        from playwright.sync_api import sync_playwright
        with sync_playwright() as pw:
            browser, page = open_page(pw, port)
            ev = page.evaluate("sfxEvents()")
            dur = page.evaluate("window.DURATION")
            browser.close()
        OUT.mkdir(exist_ok=True)
        (OUT / "events.json").write_text(json.dumps({"duration": dur, "events": ev}, indent=1))
        print(len(ev), "events ->", OUT / "events.json")
        return

    if args.mode == "stills":
        from playwright.sync_api import sync_playwright
        d = OUT / "stills"
        d.mkdir(parents=True, exist_ok=True)
        with sync_playwright() as pw:
            browser, page = open_page(pw, port)
            for t in args.times:
                page.evaluate(f"renderAt({t})")
                page.locator("#stage").screenshot(path=str(d / f"t{t:06.2f}.png"))
                print(d / f"t{t:06.2f}.png")
            browser.close()
        return

    from playwright.sync_api import sync_playwright
    with sync_playwright() as pw:
        browser, page = open_page(pw, port)
        duration = page.evaluate("window.DURATION")
        browser.close()
    n = int(round(duration * args.fps))
    frame_dir = OUT / "frames"
    frame_dir.mkdir(parents=True, exist_ok=True)
    for p in frame_dir.glob("*.png"):
        p.unlink()
    chunks = [list(range(i, n, args.workers)) for i in range(args.workers)]
    procs = [Process(target=worker, args=(port, c, args.fps, frame_dir)) for c in chunks]
    for p in procs:
        p.start()
    for p in procs:
        p.join()
    if any(p.exitcode for p in procs):
        sys.exit("a render worker failed")

    import imageio_ffmpeg
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    common = ["-framerate", str(args.fps), "-i", str(frame_dir / "f%05d.png")]
    enc = ["-c:v", "libx264", "-preset", "slow", "-crf", "14", "-pix_fmt", "yuv420p",
           "-profile:v", "high", "-movflags", "+faststart", "-color_primaries", "bt709",
           "-color_trc", "bt709", "-colorspace", "bt709"]
    silent = OUT / "drone-command-promo-silent.mp4"
    subprocess.run([ff, "-y", "-loglevel", "error", *common, *enc, str(silent)], check=True)
    print(silent)
    if os.path.exists(args.audio):
        final = OUT / "drone-command-promo.mp4"
        subprocess.run([ff, "-y", "-loglevel", "error", "-i", str(silent), "-i", args.audio,
                        "-c:v", "copy", "-c:a", "aac", "-b:a", "256k", "-shortest",
                        "-movflags", "+faststart", str(final)], check=True)
        print(final)


if __name__ == "__main__":
    main()
