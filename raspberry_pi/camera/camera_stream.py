"""
Camera Stream Module
Captures and streams video from Pi Camera

This implementation:
- Only requires picamera to enable the camera (cv2 is optional).
- Captures JPEG frames directly into memory (io.BytesIO) using the camera's JPEG encoder
  which is efficient for continuous streaming over network.
- Provides a background streaming thread with a callback for each frame.
- Provides single-frame capture helpers for compatibility.
"""
import logging
import time
import io
import threading
from datetime import datetime
from typing import Callable, Optional

try:
    from picamera import PiCamera
    # PiRGBArray is only needed if someone wants a numpy array frame
    from picamera.array import PiRGBArray
    PICAMERA_AVAILABLE = True
except Exception:
    PICAMERA_AVAILABLE = False

# cv2 is optional. If present, callers can still use numpy paths.
try:
    import cv2  # type: ignore
    CV2_AVAILABLE = True
except Exception:
    CV2_AVAILABLE = False

logger = logging.getLogger('camera_stream')


class CameraStream:
    """Pi Camera video streaming helper"""
    def __init__(self, config: dict):
        """
        Args:
            config: dict containing keys:
                - resolution: [width, height] (default: [640, 480])
                - fps: frames per second (default: 5)
                - quality: jpeg quality 0-100 (default: 75)
                - rotation: camera rotation degrees (default: 0)
        """
        self.config = config or {}
        self.resolution = tuple(self.config.get('resolution', (640, 480)))
        self.fps = int(self.config.get('fps', 5) or 5)
        self.quality = int(self.config.get('quality', 75) or 75)
        self.rotation = int(self.config.get('rotation', 0) or 0)

        self.camera: Optional[PiCamera] = None
        self.raw_capture: Optional[PiRGBArray] = None
        self.enabled = False

        # Background stream control
        self._stream_thread: Optional[threading.Thread] = None
        self._stream_stop_event = threading.Event()
        self._stream_lock = threading.Lock()

        if not PICAMERA_AVAILABLE:
            logger.warning("PiCamera not available; camera disabled")
            self.enabled = False
            return

        try:
            self.camera = PiCamera()
            self.camera.resolution = self.resolution
            self.camera.framerate = max(1, self.fps)
            self.camera.rotation = self.rotation

            # A raw array is kept only for optional numpy consumers
            try:
                self.raw_capture = PiRGBArray(self.camera, size=self.resolution)
            except Exception:
                self.raw_capture = None

            # Warm up
            time.sleep(1.5)
            self.enabled = True
            logger.info("Camera initialized: %sx%s @ %dfps (quality=%d)",
                        self.resolution[0], self.resolution[1], self.fps, self.quality)
        except Exception as exc:
            logger.exception("Failed to initialize PiCamera: %s", exc)
            self.enabled = False
            if self.camera:
                try:
                    self.camera.close()
                except Exception:
                    pass
                self.camera = None

    def capture_frame_jpeg(self) -> Optional[bytes]:
        """Capture a single JPEG frame and return raw JPEG bytes (no base64)."""
        if not self.enabled or not self.camera:
            return None

        stream = io.BytesIO()
        try:
            # Capture directly to JPEG bytes using camera encoder (fast)
            self.camera.capture(stream, format='jpeg', use_video_port=True, quality=self.quality)
            data = stream.getvalue()
            return data
        except Exception as exc:
            logger.error("Error capturing JPEG frame: %s", exc)
            return None
        finally:
            try:
                stream.close()
            except Exception:
                pass

    def capture_frame_numpy(self):
        """
        Capture a single frame into a numpy array (BGR) if possible.
        Returns: numpy array or None
        NOTE: This requires picamera.array.PiRGBArray and is slower than direct JPEG capture.
        """
        if not self.enabled or not self.camera or not self.raw_capture:
            return None

        try:
            self.raw_capture.truncate(0)
            self.camera.capture(self.raw_capture, format='bgr', use_video_port=True)
            frame = self.raw_capture.array
            return frame
        except Exception as exc:
            logger.error("Error capturing numpy frame: %s", exc)
            return None

    # ----------------------------
    # Background continuous stream
    # ----------------------------
    def start_background_stream(self, frame_callback: Callable[[bytes, str], None], fps: Optional[int] = None):
        """
        Start a background thread capturing frames continuously and invoking frame_callback(jpeg_bytes, timestamp_iso).
        If a stream is already running it will be restarted.

        Args:
            frame_callback: callable receiving (jpeg_bytes, timestamp_iso)
            fps: override FPS for this background stream
        """
        if not callable(frame_callback):
            raise ValueError("frame_callback must be callable")

        with self._stream_lock:
            self.stop_background_stream()
            self._stream_stop_event.clear()
            target_fps = int(fps or self.fps or 5)
            self._stream_thread = threading.Thread(
                target=self._stream_worker,
                args=(frame_callback, target_fps),
                daemon=True
            )
            self._stream_thread.start()
            logger.info("Background camera stream started (fps=%d)", target_fps)

    def _stream_worker(self, frame_callback: Callable[[bytes, str], None], fps: int):
        interval = 1.0 / max(1, fps)
        logger.debug("Stream worker running, interval=%.3fs", interval)

        # We capture JPEG frames directly; this avoids extra encoding overhead
        while not self._stream_stop_event.is_set():
            start = time.time()
            try:
                jpeg = self.capture_frame_jpeg()
                if jpeg:
                    timestamp = datetime.utcnow().isoformat()
                    try:
                        frame_callback(jpeg, timestamp)
                    except Exception as cb_exc:
                        logger.debug("Frame callback error: %s", cb_exc)
                else:
                    logger.debug("No frame captured this iteration")
            except Exception as exc:
                logger.exception("Exception in camera stream worker: %s", exc)

            elapsed = time.time() - start
            sleep_for = interval - elapsed
            if sleep_for > 0:
                # allow responsive shutdown
                self._stream_stop_event.wait(sleep_for)

        logger.info("Background camera stream worker stopped")

    def stop_background_stream(self):
        """Request the background stream thread to stop and wait briefly."""
        with self._stream_lock:
            if self._stream_thread and self._stream_thread.is_alive():
                self._stream_stop_event.set()
                self._stream_thread.join(timeout=1.5)
            self._stream_thread = None
            self._stream_stop_event.clear()

    def stop(self):
        """Stop everything and close camera"""
        try:
            self.stop_background_stream()
        except Exception:
            pass

        if self.camera:
            try:
                self.camera.close()
            except Exception:
                pass
            self.camera = None
            logger.info("Camera stopped")