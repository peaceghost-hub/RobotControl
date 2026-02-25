"""
Moondream AI Vision Engine — Native Torch Path
================================================

Runs Moondream 2 VLM on the dashboard server to analyse camera frames
and provide AI-powered scene understanding for robot navigation.

Architecture
------------
- Uses the NATIVE torch inference path (MoondreamModel + load_weights_into_model)
  instead of the HuggingFace transformers wrapper.  This fixes:
  • Empty responses (NaN from uninitialised tau params)
  • 280× slower bfloat16 on CPU (forced float32)
  • HF wrapper overhead

- Model loads in a background thread (non-blocking server startup).
- Inference runs in a dedicated daemon thread (non-blocking Flask).
- Grabs the latest JPEG frame from the shared ``latest_data`` store.
- Results are broadcast to all connected clients via SocketIO.
- Multiple analysis modes: scene description, obstacle detection,
  direction advice, terrain analysis, object detection, and custom VQA.

Deployment
----------
Runs on the **dashboard server** (laptop / desktop), *not* on the Pi.
The 2B-param model needs ~8 GB RAM in float32.
The Pi cannot run it without freezing — violates the GOLDEN RULE.

GOLDEN RULE: NO FREEZING — every operation is non-blocking.
"""

from __future__ import annotations

import io
import os
import time
import logging
import threading
from typing import Optional, Dict, Any, Callable, List

logger = logging.getLogger("ai_vision")

# ── Lazy heavy imports (dashboard still works if torch is missing) ────
_TORCH_AVAILABLE = False
_PIL_AVAILABLE = False

try:
    import torch
    _TORCH_AVAILABLE = True
    # Use all CPU cores for parallelism.
    torch.set_num_threads(os.cpu_count() or 4)
    # Prevent torch._dynamo from wasting time on compilation attempts.
    torch._dynamo.config.suppress_errors = True  # type: ignore[attr-defined]
except ImportError:
    torch = None  # type: ignore[assignment]

try:
    from PIL import Image
    _PIL_AVAILABLE = True
except ImportError:
    Image = None  # type: ignore[misc,assignment]

# ── Robot physical parameters ────────────────────────────────────────
ROBOT_HEIGHT_CM = 20
ROBOT_WIDTH_CM = 20
WHEEL_RADIUS_CM = 2
ACTION_DISTANCE_CM = 20

# Moondream vision encoder crop size.  Pre-resizing the image so its
# longest edge matches this avoids generating many crops (up to 12),
# which dominate inference time on CPU.
_VISION_CROP_PX = 378

# ── Navigation prompt templates ──────────────────────────────────────
PROMPTS: Dict[str, str] = {
    "scene": (
        "You are the AI eyes of a ground-based robot. Describe what you "
        "see in 2-3 sentences. Focus on: terrain type, obstacles, path "
        "clearance, and any notable objects or hazards."
    ),
    "obstacles": (
        "Is the path ahead clear for a ground robot to drive forward? "
        "Answer YES or NO first, then briefly list any obstacles you see."
    ),
    "direction": (
        "You are navigating a ground robot. Based on what you see, what "
        "direction should it go? Answer with one of: FORWARD, LEFT, RIGHT, "
        "STOP. Then explain in one sentence."
    ),
    "terrain": (
        "What type of terrain or surface do you see? Is it suitable for "
        "a wheeled ground robot? Answer briefly in 1-2 sentences."
    ),
    "navigate": (
        "You are the autonomous AI vision system for a small wheeled ground robot. "
        f"Robot specs: height {ROBOT_HEIGHT_CM}cm, width {ROBOT_WIDTH_CM}cm, "
        f"wheel radius {WHEEL_RADIUS_CM}cm, reaction distance {ACTION_DISTANCE_CM}cm. "
        f"The camera is at {ROBOT_HEIGHT_CM}cm above ground (low perspective). "
        "Analyze this scene for safe navigation. "
        "Respond in EXACTLY this format (three lines, nothing else):\n"
        "SAFETY: SAFE or CAUTION or DANGER\n"
        "DIRECTION: FORWARD or LEFT or RIGHT or STOP\n"
        "REASON: one brief sentence\n\n"
        "Rules:\n"
        f"- SAFE + FORWARD: clear path, no obstacles within {ACTION_DISTANCE_CM}cm\n"
        "- CAUTION + FORWARD: minor obstacles visible but path is passable\n"
        "- CAUTION + LEFT or RIGHT: partial obstruction, navigate around it\n"
        f"- DANGER + STOP: path fully blocked within {ACTION_DISTANCE_CM}cm\n"
        "- DANGER + LEFT or RIGHT: obstacle ahead but space to steer around\n"
        f"- Objects taller than {WHEEL_RADIUS_CM}cm on the ground can block the wheels\n"
        f"- Objects wider than {ROBOT_WIDTH_CM}cm blocking the path cannot be passed\n"
        "- Holes, drops, steps, or uneven terrain are hazards for small wheels\n"
        "- Consider the low ground-level camera perspective"
    ),
}

# ── Default configuration ────────────────────────────────────────────
DEFAULT_MODEL_ID = os.getenv("MOONDREAM_MODEL_ID", "vikhyatk/moondream2")
DEFAULT_REVISION = os.getenv("MOONDREAM_REVISION", "2025-06-21")
DEFAULT_INTERVAL = 5.0  # seconds between auto-analyses

# Weight file path — use HF cache if available
_DEFAULT_WEIGHTS_PATH = os.path.expanduser(
    os.getenv(
        "MOONDREAM_WEIGHTS",
        "~/.cache/huggingface/hub/models--vikhyatk--moondream2/"
        "snapshots/9a7d4024050840e001defacec2b00727e89149e6/model.safetensors",
    )
)


# ======================================================================
#  Null stub — used when dependencies are missing
# ======================================================================
class _NullVision:
    """Harmless stub so Flask routes never need ``if vision is None``."""

    status = "unavailable"
    enabled = False
    is_ready = False
    last_result = None

    auto_drive = False
    last_nav = None
    drive_count = 0
    base_nav_mode = "none"

    def load_model(self) -> None: ...
    def unload_model(self) -> None: ...
    def set_enabled(self, _e: bool) -> None: ...
    def set_mode(self, *_a: Any) -> None: ...
    def set_interval(self, _s: float) -> None: ...
    def set_auto_drive(self, _e: bool) -> None: ...
    def set_base_nav(self, _m: str) -> None: ...
    def query_once(self, *_a: Any) -> None: return None
    def detect_once(self, *_a: Any) -> None: return None

    def get_status(self) -> Dict[str, Any]:
        return {
            "status": "unavailable",
            "enabled": False,
            "mode": "scene",
            "interval": DEFAULT_INTERVAL,
            "device": "none",
            "model_id": DEFAULT_MODEL_ID,
            "analysis_count": 0,
            "detect_target": "",
            "custom_prompt": "",
            "error": "AI Vision dependencies not installed (torch, safetensors, tokenizers)",
            "last_result": None,
            "torch_available": _TORCH_AVAILABLE,
            "transformers_available": False,
            "auto_drive": False,
            "last_nav": None,
            "drive_count": 0,
            "base_nav_mode": "none",
            "robot_params": {
                "height_cm": ROBOT_HEIGHT_CM,
                "width_cm": ROBOT_WIDTH_CM,
                "wheel_radius_cm": WHEEL_RADIUS_CM,
                "action_distance_cm": ACTION_DISTANCE_CM,
            },
        }


NullVision = _NullVision  # public alias


# ======================================================================
#  Main engine — native torch path
# ======================================================================
class MoondreamVision:
    """Non-blocking Moondream 2 wrapper using the native torch inference path.

    Key difference from the old HF-based approach:
    - Loads MoondreamModel directly (no transformers AutoModel)
    - Uses load_weights_into_model() from safetensors
    - Forces float32 (bfloat16 is ~280× slower on CPU)
    - Disables flex_attention (not needed on CPU)
    - Initialises tau params with zeros (prevents NaN logits)
    - Uses dense config (no MoE — matches the 2025-06-21 weights)
    """

    def __init__(
        self,
        weights_path: Optional[str] = None,
        device: str = "auto",
    ):
        self._weights_path = weights_path or _DEFAULT_WEIGHTS_PATH
        self._model_id = DEFAULT_MODEL_ID
        self._preferred_device = device

        self._model: Any = None
        self._device: Optional[str] = None
        self._dtype: Any = None

        self._status = "not_loaded"
        self._error_msg = ""
        self._lock = threading.Lock()

        # Analysis state
        self._enabled = False
        self._interval = DEFAULT_INTERVAL
        self._mode = "scene"
        self._detect_target = "obstacle"
        self._custom_prompt = ""
        self._last_result: Optional[Dict] = None
        self._analysis_count = 0
        self._desired_enabled = False

        # Background analysis thread
        self._analysis_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Auto-drive
        self._auto_drive = False
        self._base_nav_mode = "none"
        self._last_nav: Optional[Dict[str, str]] = None
        self._drive_count = 0

        # Injected by app.py after construction
        self._frame_provider: Optional[Callable[[], Optional[bytes]]] = None
        self._emit_fn: Optional[Callable[[str, Dict], None]] = None
        self._drive_command_fn: Optional[Callable[[Dict], None]] = None

        # Pre-encoded image cache (avoid re-encoding same frame)
        self._cached_image_hash: Optional[int] = None
        self._cached_encoded: Any = None

        # Sanity-check dependencies
        if not _TORCH_AVAILABLE:
            self._status = "unavailable"
            self._error_msg = "Missing: torch. Install with:  pip install torch"
            logger.warning("AI Vision unavailable — %s", self._error_msg)
        elif not os.path.exists(self._weights_path):
            self._error_msg = (
                f"Weights not found at {self._weights_path}. "
                "Download the model first."
            )
            logger.warning("AI Vision — %s", self._error_msg)

    # ─── Properties ───────────────────────────────────────────────────
    @property
    def status(self) -> str:
        return self._status

    @property
    def is_ready(self) -> bool:
        return self._status == "ready"

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def last_result(self) -> Optional[Dict]:
        return self._last_result

    @property
    def auto_drive(self) -> bool:
        return self._auto_drive

    @property
    def last_nav(self) -> Optional[Dict[str, str]]:
        return self._last_nav

    @property
    def base_nav_mode(self) -> str:
        return self._base_nav_mode

    def set_auto_drive(self, enabled: bool) -> None:
        self._auto_drive = enabled
        if enabled:
            if self._mode != "navigate":
                self._mode = "navigate"
            if not self._enabled:
                self.set_enabled(True)
        logger.info("Auto-drive → %s  (base_nav=%s)",
                    "ON" if enabled else "OFF", self._base_nav_mode)
        self._broadcast_status()

    def set_base_nav(self, mode: str) -> None:
        prev = self._base_nav_mode
        self._base_nav_mode = mode if mode in ("manual", "waypoint") else "none"
        if prev != self._base_nav_mode:
            logger.info("Base nav: %s → %s", prev, self._base_nav_mode)
        self._broadcast_status()

    # ─── Navigation decision parser ───────────────────────────────────
    @staticmethod
    def _parse_nav_decision(text: str) -> Dict[str, str]:
        safety = "DANGER"
        direction = "STOP"
        reason = "Could not parse AI response"

        if not text:
            return {"safety": safety, "direction": direction, "reason": reason}

        for line in text.strip().splitlines():
            upper = line.strip().upper()
            if upper.startswith("SAFETY:"):
                val = upper.split(":", 1)[1].strip()
                if val in ("SAFE", "CAUTION", "DANGER"):
                    safety = val
            elif upper.startswith("DIRECTION:"):
                val = upper.split(":", 1)[1].strip()
                if val in ("FORWARD", "LEFT", "RIGHT", "STOP"):
                    direction = val
            elif upper.startswith("REASON:"):
                reason = line.strip().split(":", 1)[1].strip()

        return {"safety": safety, "direction": direction, "reason": reason}

    def _send_drive_command(self, nav: Dict[str, str]) -> None:
        if not self._drive_command_fn:
            return
        if self._base_nav_mode == "none":
            return

        if self._base_nav_mode == "waypoint":
            if nav["safety"] == "SAFE" and nav["direction"] == "FORWARD":
                logger.debug("AI: SAFE+FORWARD during waypoint nav — letting NavController drive")
                return

        try:
            self._drive_command_fn({
                "command": "AI_DRIVE",
                "payload": {
                    "direction": nav["direction"],
                    "safety": nav["safety"],
                    "reason": nav["reason"],
                    "base_nav": self._base_nav_mode,
                },
            })
            self._drive_count += 1
            logger.info(
                "AI_DRIVE → %s (safety=%s, base=%s) — %s",
                nav["direction"], nav["safety"],
                self._base_nav_mode, nav["reason"],
            )
        except Exception as exc:
            logger.error("Failed to send AI_DRIVE: %s", exc)

    # ─── Model lifecycle ──────────────────────────────────────────────
    def load_model(self) -> None:
        if self._status in ("loading", "ready", "unavailable"):
            return
        self._status = "loading"
        self._error_msg = ""
        self._broadcast_status()
        threading.Thread(
            target=self._load_model_sync, daemon=True, name="moondream-loader"
        ).start()
        logger.info("Moondream model load started (background)…")

    def _load_model_sync(self) -> None:
        """Load the model using the native torch path — runs in daemon thread."""
        try:
            import sys
            # Ensure md_reference package is importable
            dashboard_dir = os.path.dirname(os.path.abspath(__file__))
            if dashboard_dir not in sys.path:
                sys.path.insert(0, dashboard_dir)

            from md_reference.config import MoondreamConfig, TextConfig
            from md_reference.moondream import MoondreamModel
            from md_reference.weights import load_weights_into_model

            logger.info("Loading weights from %s …", self._weights_path)

            # Pick device + dtype
            if self._preferred_device == "auto":
                if torch.cuda.is_available():
                    self._device = "cuda"
                    self._dtype = torch.float16
                elif hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
                    self._device = "mps"
                    self._dtype = torch.float16
                else:
                    self._device = "cpu"
                    self._dtype = torch.float32  # bf16 is ~280× slower on CPU
            else:
                self._device = self._preferred_device
                self._dtype = (
                    torch.float16 if self._device != "cpu" else torch.float32
                )

            logger.info("Device: %s   dtype: %s", self._device, self._dtype)

            # Build model — NO MoE (the 2025-06-21 weights are dense)
            t0 = time.time()
            text_cfg = TextConfig(moe=None)
            config = MoondreamConfig(text=text_cfg)
            model = MoondreamModel(config, dtype=self._dtype, setup_caches=True)
            logger.info("Model structure built in %.1fs", time.time() - t0)

            # Load weights from safetensors
            t0 = time.time()
            load_weights_into_model(self._weights_path, model)
            logger.info("Weights loaded in %.1fs", time.time() - t0)

            # Convert to target dtype (weights are stored as bf16)
            t0 = time.time()
            model = model.to(device=self._device, dtype=self._dtype)
            model.eval()

            # Disable flex_attention on CPU (not well supported)
            if self._device == "cpu":
                model.use_flex_decoding = False

            # Rebuild KV caches in the correct dtype
            model._setup_caches()
            logger.info("Converted to %s in %.1fs", self._dtype, time.time() - t0)

            with self._lock:
                self._model = model
                self._status = "ready"

            logger.info("✓ Moondream ready on %s (%s)", self._device, self._dtype)
            if self._desired_enabled:
                self.set_enabled(True)
            else:
                self._broadcast_status()

        except Exception as exc:
            with self._lock:
                self._status = "error"
                self._error_msg = str(exc)
            logger.error("Moondream load failed: %s", exc, exc_info=True)
            self._broadcast_status()

    def unload_model(self) -> None:
        self.set_enabled(False)
        with self._lock:
            if self._model is not None:
                del self._model
                self._model = None
                self._cached_encoded = None
                self._cached_image_hash = None
                if _TORCH_AVAILABLE and torch.cuda.is_available():
                    torch.cuda.empty_cache()
            self._status = "not_loaded"
            self._error_msg = ""
        logger.info("Moondream model unloaded")
        self._broadcast_status()

    # ─── Analysis control ─────────────────────────────────────────────
    def set_enabled(self, enabled: bool) -> None:
        self._desired_enabled = enabled

        if enabled and self._status != "ready":
            if self._status in ("not_loaded", "error"):
                self.load_model()
            return

        self._enabled = enabled

        if enabled:
            self._start_analysis_thread()
            logger.info(
                "AI Vision ON  (interval=%.1fs  mode=%s)", self._interval, self._mode
            )
        else:
            self._stop_event.set()
            logger.info("AI Vision OFF")

        self._broadcast_status()

    def set_mode(
        self,
        mode: str,
        detect_target: Optional[str] = None,
        custom_prompt: Optional[str] = None,
    ) -> None:
        if mode in PROMPTS or mode in ("detect", "custom"):
            self._mode = mode
        if detect_target is not None:
            self._detect_target = detect_target
        if custom_prompt is not None:
            self._custom_prompt = custom_prompt
        logger.info("AI mode → %s", mode)

    def set_interval(self, seconds: float) -> None:
        self._interval = max(1.0, min(30.0, seconds))
        logger.info("AI interval → %.1fs", self._interval)

    def _start_analysis_thread(self) -> None:
        if self._analysis_thread is None or not self._analysis_thread.is_alive():
            self._stop_event.clear()
            self._analysis_thread = threading.Thread(
                target=self._analysis_loop, daemon=True,
                name="moondream-analysis",
            )
            self._analysis_thread.start()

    # ─── Background analysis loop ─────────────────────────────────────
    def _analysis_loop(self) -> None:
        logger.info("Analysis loop started")
        while not self._stop_event.is_set():
            if not self._enabled or self._status != "ready":
                self._stop_event.wait(0.5)
                continue

            try:
                frame_bytes = self._frame_provider() if self._frame_provider else None
                if frame_bytes is None:
                    self._stop_event.wait(1.0)
                    continue

                # Let the UI know inference is in progress
                if self._emit_fn:
                    self._emit_fn("ai_vision_update", {
                        "mode": self._mode,
                        "response": "Analyzing…",
                        "inference_time": None,
                        "count": self._analysis_count,
                    })

                result = self._run_analysis(frame_bytes)
                if result:
                    with self._lock:
                        self._last_result = result
                        self._analysis_count += 1
                    if self._emit_fn:
                        self._emit_fn("ai_vision_update", result)

            except Exception as exc:
                logger.error("Analysis loop error: %s", exc)

            self._stop_event.wait(self._interval)

        logger.info("Analysis loop stopped")

    # ─── Core model calls ─────────────────────────────────────────────
    # TextSamplingSettings: temperature, max_tokens, top_p, variant
    _NAV_SETTINGS: dict = {"temperature": 0, "max_tokens": 100}
    _SCENE_SETTINGS: dict = {"temperature": 0, "max_tokens": 256}
    _CUSTOM_SETTINGS: dict = {"temperature": 0.5, "max_tokens": 512}

    def _query(self, image: Any, prompt: str, settings: Dict[str, Any]) -> Dict[str, Any]:
        """Run model.query() with the native path."""
        result = self._model.query(
            image, prompt, reasoning=False, stream=False, settings=settings or {}
        )
        if isinstance(result, dict):
            return result
        return {"answer": str(result)}

    def _detect(self, image: Any, target: str) -> Dict[str, Any]:
        """Run model.detect() with the native path."""
        try:
            result = self._model.detect(image, target)
            if isinstance(result, dict):
                return result
        except Exception as exc:
            logger.warning("detect() failed, falling back to query: %s", exc)

        # Fallback: use query
        fallback = self._query(
            image,
            f"Do you see any {target}? Count and describe briefly.",
            self._SCENE_SETTINGS,
        )
        if isinstance(fallback, dict) and "answer" in fallback:
            return {"objects": [], "response": fallback["answer"]}
        return {"objects": []}

    # ─── Image pre-processing ─────────────────────────────────────────
    @staticmethod
    def _resize_for_model(image: Any) -> Any:
        """Shrink image so its longest edge ≤ _VISION_CROP_PX.

        This avoids Moondream creating many crops (up to 12) from a
        large camera frame.  Fewer crops → dramatically faster vision
        encoding on CPU.  Quality loss is negligible for VQA.
        """
        w, h = image.size
        longest = max(w, h)
        if longest <= _VISION_CROP_PX:
            return image
        scale = _VISION_CROP_PX / longest
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))
        return image.resize((new_w, new_h), Image.LANCZOS)

    def _run_analysis(self, frame_bytes: bytes) -> Optional[Dict]:
        """Run one analysis on a JPEG frame.  Called from bg thread."""
        if self._model is None or not frame_bytes:
            return None
        try:
            image = Image.open(io.BytesIO(frame_bytes)).convert("RGB")
            image = self._resize_for_model(image)
            t0 = time.time()
            logger.info("AI inference started  (mode=%s, size=%s)",
                        self._mode, image.size)

            result: Dict[str, Any] = {
                "timestamp": time.time(),
                "mode": self._mode,
                "count": self._analysis_count,
            }

            with torch.inference_mode():

                if self._mode == "detect":
                    detections = self._detect(image, self._detect_target)
                    objects: List[Dict] = detections.get("objects", []) if isinstance(detections, dict) else []
                    result["detections"] = objects
                    result["detect_target"] = self._detect_target
                    result["num_objects"] = len(objects)
                    resp_text = None
                    if isinstance(detections, dict):
                        resp_text = detections.get("response") or detections.get("text")
                    result["response"] = resp_text or (
                        f"Detected {len(objects)} '{self._detect_target}' object(s)"
                    )

                elif self._mode == "navigate":
                    prompt = PROMPTS["navigate"]
                    answer = self._query(image, prompt, self._NAV_SETTINGS)
                    raw = answer.get("answer", "")
                    result["response"] = raw
                    result["prompt"] = prompt

                    nav = self._parse_nav_decision(raw)
                    result["nav_decision"] = nav
                    self._last_nav = nav

                    if self._auto_drive:
                        self._send_drive_command(nav)
                        result["drive_sent"] = True
                        result["drive_count"] = self._drive_count

                elif self._mode == "custom" and self._custom_prompt:
                    answer = self._query(image, self._custom_prompt, self._CUSTOM_SETTINGS)
                    result["response"] = answer.get("answer", "")
                    result["prompt"] = self._custom_prompt

                else:
                    prompt = PROMPTS.get(self._mode, PROMPTS["scene"])
                    answer = self._query(image, prompt, self._SCENE_SETTINGS)
                    result["response"] = answer.get("answer", "")
                    result["prompt"] = prompt

            result["inference_time"] = round(time.time() - t0, 2)
            logger.info(
                "AI [%s] %.1fs — %s",
                self._mode, result["inference_time"],
                (result.get("response") or "")[:120],
            )
            return result

        except Exception as exc:
            logger.error("Inference error: %s", exc, exc_info=True)
            return None

    # ─── One-shot helpers (manual from API) ───────────────────────────
    def query_once(self, frame_bytes: bytes, prompt: str) -> Optional[Dict]:
        if self._model is None or not frame_bytes:
            return None
        try:
            image = Image.open(io.BytesIO(frame_bytes)).convert("RGB")
            image = self._resize_for_model(image)
            t0 = time.time()
            with torch.inference_mode():
                answer = self._query(image, prompt, self._SCENE_SETTINGS)
            return {
                "timestamp": time.time(),
                "mode": "manual",
                "prompt": prompt,
                "response": answer.get("answer", ""),
                "inference_time": round(time.time() - t0, 2),
                "count": self._analysis_count,
            }
        except Exception as exc:
            logger.error("Manual query error: %s", exc, exc_info=True)
            return None

    def detect_once(self, frame_bytes: bytes, target: str) -> Optional[Dict]:
        if self._model is None or not frame_bytes:
            return None
        try:
            image = Image.open(io.BytesIO(frame_bytes)).convert("RGB")
            image = self._resize_for_model(image)
            t0 = time.time()
            with torch.inference_mode():
                detections = self._detect(image, target)
            objects = detections.get("objects", []) if isinstance(detections, dict) else []
            resp_text = detections.get("response") if isinstance(detections, dict) else None
            return {
                "timestamp": time.time(),
                "mode": "detect",
                "detect_target": target,
                "detections": objects,
                "num_objects": len(objects),
                "response": resp_text or f"Detected {len(objects)} '{target}' object(s)",
                "inference_time": round(time.time() - t0, 2),
                "count": self._analysis_count,
            }
        except Exception as exc:
            logger.error("Detection error: %s", exc, exc_info=True)
            return None

    # ─── Status ───────────────────────────────────────────────────────
    def get_status(self) -> Dict[str, Any]:
        return {
            "status": self._status,
            "enabled": self._enabled,
            "mode": self._mode,
            "interval": self._interval,
            "device": self._device or "none",
            "model_id": self._model_id,
            "analysis_count": self._analysis_count,
            "detect_target": self._detect_target,
            "custom_prompt": self._custom_prompt,
            "error": self._error_msg,
            "last_result": self._last_result,
            "torch_available": _TORCH_AVAILABLE,
            "transformers_available": True,  # compat — native path always available
            "auto_drive": self._auto_drive,
            "base_nav_mode": self._base_nav_mode,
            "last_nav": self._last_nav,
            "drive_count": self._drive_count,
            "robot_params": {
                "height_cm": ROBOT_HEIGHT_CM,
                "width_cm": ROBOT_WIDTH_CM,
                "wheel_radius_cm": WHEEL_RADIUS_CM,
                "action_distance_cm": ACTION_DISTANCE_CM,
            },
        }

    # ─── Internal helpers ─────────────────────────────────────────────
    def _broadcast_status(self) -> None:
        if self._emit_fn:
            try:
                self._emit_fn("ai_vision_status", self.get_status())
            except Exception as exc:
                logger.warning("_broadcast_status failed: %s", exc)
