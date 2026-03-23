#!/usr/bin/env python3
"""
Local MJPEG Camera Viewer — pure Tkinter + Pillow, zero extra dependencies.

Connects to the Pi's already-running MJPEG server at localhost:8081/stream
and displays frames in a desktop window.  Ideal for viewing over VNC.

Usage:
    python3 local_viewer.py              # defaults: localhost:8081, 640×480
    python3 local_viewer.py --port 8081 --scale 2
    python3 local_viewer.py --host 192.168.1.50 --port 8081

Press Q or Escape to quit.  Close the window to quit.
"""

import argparse
import io
import logging
import socket
import threading
import time
import tkinter as tk
from http.client import HTTPConnection
from tkinter import Label, StringVar
from PIL import Image, ImageTk  # Pillow — ships with Raspbian by default

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("local_viewer")


# ─── MJPEG stream reader ─────────────────────────────────────────────────────

def mjpeg_frame_generator(host: str, port: int, path: str, timeout: float = 5.0):
    """Yield raw JPEG bytes from an MJPEG multipart HTTP stream."""
    while True:
        try:
            conn = HTTPConnection(host, port, timeout=timeout)
            conn.request("GET", path)
            resp = conn.getresponse()
            if resp.status != 200:
                log.warning("HTTP %s from MJPEG server — retrying in 2 s", resp.status)
                conn.close()
                time.sleep(2)
                continue

            buf = b""
            while True:
                chunk = resp.read(4096)
                if not chunk:
                    break
                buf += chunk

                # Find JPEG boundaries (SOI = FF D8, EOI = FF D9)
                while True:
                    soi = buf.find(b"\xff\xd8")
                    if soi == -1:
                        buf = buf[-1:]
                        break
                    eoi = buf.find(b"\xff\xd9", soi + 2)
                    if eoi == -1:
                        break
                    jpeg = buf[soi : eoi + 2]
                    buf = buf[eoi + 2 :]
                    yield jpeg

        except (OSError, socket.timeout, ConnectionRefusedError) as exc:
            log.warning("Stream connection lost (%s) — reconnecting in 2 s", exc)
        except Exception as exc:
            log.error("Unexpected error: %s — retrying in 2 s", exc)
        finally:
            try:
                conn.close()
            except Exception:
                pass
        time.sleep(2)


# ─── Tkinter viewer ──────────────────────────────────────────────────────────

class MJPEGViewer:
    """Tk window that displays MJPEG frames from a background thread."""

    def __init__(self, host: str, port: int, path: str, scale: float = 1.0):
        self.host = host
        self.port = port
        self.path = path
        self.scale = scale
        self._running = True
        self._frame_lock = threading.Lock()
        self._latest_tk_image = None
        self._fps_count = 0
        self._fps_time = time.monotonic()
        self._fps_val = 0.0

        # ── Build the window ──
        self.root = tk.Tk()
        self.root.title("Robot Camera — Local Viewer")
        self.root.configure(bg="black")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.bind("<Escape>", lambda e: self._on_close())
        self.root.bind("<q>", lambda e: self._on_close())
        self.root.bind("<Q>", lambda e: self._on_close())

        self._status_var = StringVar(value="Connecting to stream...")
        self._status_label = Label(
            self.root, textvariable=self._status_var,
            fg="#22c55e", bg="black", font=("monospace", 10), anchor="w"
        )
        self._status_label.pack(side="bottom", fill="x", padx=6, pady=2)

        self._canvas = Label(self.root, bg="black")
        self._canvas.pack(expand=True, fill="both")

        # ── Start reader thread ──
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        # ── Schedule periodic UI refresh ──
        self.root.after(30, self._refresh_ui)

    def _reader_loop(self):
        for jpeg_bytes in mjpeg_frame_generator(self.host, self.port, self.path):
            if not self._running:
                return
            try:
                img = Image.open(io.BytesIO(jpeg_bytes))
                if self.scale != 1.0:
                    w, h = img.size
                    img = img.resize(
                        (int(w * self.scale), int(h * self.scale)),
                        Image.LANCZOS,
                    )
                tk_img = ImageTk.PhotoImage(img)
                with self._frame_lock:
                    self._latest_tk_image = tk_img
                    self._fps_count += 1
            except Exception:
                pass

    def _refresh_ui(self):
        if not self._running:
            return
        with self._frame_lock:
            img = self._latest_tk_image
        if img is not None:
            self._canvas.configure(image=img)
            self._canvas.image = img

        now = time.monotonic()
        elapsed = now - self._fps_time
        if elapsed >= 1.0:
            with self._frame_lock:
                self._fps_val = self._fps_count / elapsed
                self._fps_count = 0
            self._fps_time = now
            self._status_var.set(
                f"Stream: {self.host}:{self.port}{self.path}  |  "
                f"{self._fps_val:.1f} FPS  |  "
                f"Press Q / Esc to quit"
            )

        self.root.after(30, self._refresh_ui)

    def _on_close(self):
        self._running = False
        self.root.destroy()

    def run(self):
        self.root.mainloop()


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Local MJPEG camera viewer (Tkinter)")
    ap.add_argument("--host", default="localhost",
                    help="MJPEG server host (default: localhost)")
    ap.add_argument("--port", type=int, default=8081,
                    help="MJPEG server port (default: 8081)")
    ap.add_argument("--path", default="/stream",
                    help="MJPEG stream path (default: /stream)")
    ap.add_argument("--scale", type=float, default=2.0,
                    help="Display scale factor (default: 2.0 = 640x480 from 320x240 source)")
    args = ap.parse_args()

    log.info("Opening viewer — %s:%d%s  (scale %.1fx)", args.host, args.port, args.path, args.scale)
    viewer = MJPEGViewer(args.host, args.port, args.path, args.scale)
    viewer.run()


if __name__ == "__main__":
    main()
