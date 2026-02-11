"""
Lightweight MJPEG streaming server for Raspberry Pi camera.

Runs a dedicated HTTP server (default port 8081) that serves a
native MJPEG multipart stream directly from the Pi camera.
This eliminates the double-relay through the dashboard:

    OLD:  Pi → HTTP POST → Dashboard → MJPEG poll → Browser
    NEW:  Pi → MJPEG stream ──────────────────────→ Browser

The dashboard can still receive frames for thumbnail/recording,
but the live video feeds directly from the Pi to the browser.
"""

import io
import logging
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Optional

logger = logging.getLogger('mjpeg_server')

# Shared state for latest frame
_frame_lock = threading.Lock()
_latest_frame: Optional[bytes] = None
_frame_event = threading.Event()


def update_frame(jpeg_bytes: bytes):
    """Called by the camera capture loop to publish a new frame."""
    global _latest_frame
    with _frame_lock:
        _latest_frame = jpeg_bytes
    _frame_event.set()


class MJPEGHandler(BaseHTTPRequestHandler):
    """HTTP handler that serves MJPEG multipart stream on /stream
    and a single JPEG snapshot on /snapshot."""

    # Suppress default logging (we use our own logger)
    def log_message(self, fmt, *args):
        logger.debug("MJPEG HTTP: " + fmt, *args)

    def do_GET(self):
        if self.path == '/stream':
            self._serve_mjpeg()
        elif self.path == '/snapshot':
            self._serve_snapshot()
        elif self.path == '/':
            # Simple HTML page that embeds the stream
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><body style="margin:0;background:#000">'
                             b'<img src="/stream" style="width:100%;height:auto">'
                             b'</body></html>')
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_snapshot(self):
        """Serve a single JPEG frame."""
        with _frame_lock:
            frame = _latest_frame
        if frame is None:
            self.send_response(503)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'No frame available')
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(frame)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(frame)

    def _serve_mjpeg(self):
        """Serve continuous MJPEG multipart stream."""
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=--frame')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('X-Accel-Buffering', 'no')
        self.end_headers()

        try:
            while True:
                # Wait for a new frame (with timeout so we can detect disconnects)
                _frame_event.wait(timeout=2.0)
                _frame_event.clear()

                with _frame_lock:
                    frame = _latest_frame
                if frame is None:
                    continue

                try:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(('Content-Length: %d\r\n\r\n' % len(frame)).encode())
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    break
        except Exception as e:
            logger.debug("MJPEG stream ended: %s", e)


class MJPEGServer:
    """Manages the MJPEG HTTP server in a background thread."""

    def __init__(self, host: str = '0.0.0.0', port: int = 8081):
        self.host = host
        self.port = port
        self._server: Optional[HTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def start(self):
        """Start the MJPEG server in a background thread."""
        try:
            self._server = HTTPServer((self.host, self.port), MJPEGHandler)
            self._server.timeout = 1  # so shutdown is responsive
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            logger.info("MJPEG server started on http://%s:%d/stream", self.host, self.port)
        except Exception as e:
            logger.error("Failed to start MJPEG server: %s", e)
            self._server = None

    def _run(self):
        """Server loop."""
        if self._server:
            self._server.serve_forever()

    def stop(self):
        """Shutdown the server."""
        if self._server:
            self._server.shutdown()
            self._server = None
            logger.info("MJPEG server stopped")

    @property
    def url(self) -> str:
        """Return the stream URL."""
        return f"http://{self.host}:{self.port}/stream"
