"""
Moondream AI Vision Engine
===========================

Runs Moondream 2 VLM on the dashboard server to analyse camera frames
and provide AI-powered scene understanding for robot navigation.

Architecture
------------
- Model loads in a background thread (non-blocking server startup).
- Inference runs in a dedicated daemon thread (non-blocking Flask).
- Grabs the latest JPEG frame from the shared ``latest_data`` store.
- Results are broadcast to all connected clients via SocketIO.
- Multiple analysis modes: scene description, obstacle detection,
  direction advice, terrain analysis, object detection, and custom VQA.

Deployment
----------
Runs on the **dashboard server** (laptop / desktop), *not* on the Pi.
The 2B-param model needs ~4 GB VRAM (GPU) or ~8 GB RAM (CPU fp32).
The Pi cannot run it without freezing — violates the GOLDEN RULE.

GOLDEN RULE: NO FREEZING — every operation is non-blocking.
"""

from __future__ import annotations

import io
import time
import logging
import threading
from typing import Optional, Dict, Any, Callable, List

logger = logging.getLogger("ai_vision")

# ── Lazy heavy imports (dashboard still works if torch is missing) ────
_TORCH_AVAILABLE = False
_TRANSFORMERS_AVAILABLE = False
_PIL_AVAILABLE = False

try:
    import torch
    _TORCH_AVAILABLE = True
except ImportError:
    torch = None  # type: ignore[assignment]

try:
    from transformers import AutoModelForCausalLM
    _TRANSFORMERS_AVAILABLE = True
except ImportError:
    AutoModelForCausalLM = None  # type: ignore[misc,assignment]

try:
    from PIL import Image
    _PIL_AVAILABLE = True
except ImportError:
    Image = None  # type: ignore[misc,assignment]

# ── Robot physical parameters ────────────────────────────────────────
# These govern what the AI tells the Pi — the conclusion it makes,
# the drive commands to be sent, all depend on the robot's dimensions.
ROBOT_HEIGHT_CM = 20        # camera mounted at this height
ROBOT_WIDTH_CM = 20         # chassis width
WHEEL_RADIUS_CM = 2         # objects > this on ground can block wheels
ACTION_DISTANCE_CM = 20     # react to obstacles within this range

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
DEFAULT_MODEL_ID = "vikhyatk/moondream2"
DEFAULT_REVISION = "2025-06-21"
DEFAULT_INTERVAL = 3.0  # seconds between auto-analyses


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
    base_nav_mode = "none"   # 'none' | 'manual' | 'waypoint'

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
            "error": "AI Vision dependencies not installed (torch, transformers)",
            "last_result": None,
            "torch_available": _TORCH_AVAILABLE,
            "transformers_available": _TRANSFORMERS_AVAILABLE,
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
#  Main engine
# ======================================================================
class MoondreamVision:
    """Non-blocking Moondream 2 wrapper for dashboard integration."""

    def __init__(
        self,
        model_id: Optional[str] = None,
        revision: Optional[str] = None,
        device: str = "auto",
    ):
        self._model_id = model_id or DEFAULT_MODEL_ID
        self._revision = revision or DEFAULT_REVISION
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

        # Background analysis thread
        self._analysis_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Auto-drive: when enabled, navigate mode pushes AI_DRIVE
        # commands to the Pi via the injected callback.
        # RULE: auto-drive only works when a base navigation method is
        # already active (manual forward or waypoint navigation).
        self._auto_drive = False
        self._base_nav_mode = "none"   # 'none' | 'manual' | 'waypoint'
        self._last_nav: Optional[Dict[str, str]] = None  # {safety, direction, reason}
        self._drive_count = 0

        # Injected by app.py after construction
        self._frame_provider: Optional[Callable[[], Optional[bytes]]] = None
        self._emit_fn: Optional[Callable[[str, Dict], None]] = None
        self._drive_command_fn: Optional[Callable[[Dict], None]] = None

        # Sanity-check dependencies
        if not _TORCH_AVAILABLE or not _TRANSFORMERS_AVAILABLE:
            self._status = "unavailable"
            missing = []
            if not _TORCH_AVAILABLE:
                missing.append("torch")
            if not _TRANSFORMERS_AVAILABLE:
                missing.append("transformers")
            self._error_msg = (
                f"Missing: {', '.join(missing)}. "
                "Install with:  pip install torch transformers"
            )
            logger.warning("AI Vision unavailable — %s", self._error_msg)

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
        """Toggle auto-drive: navigate mode sends commands to the Pi.

        RULE: Auto-drive can ONLY be enabled when a base navigation
        method is already active (manual forward or waypoint nav).
        It never works alone or as the first action.
        """
        if enabled and self._base_nav_mode == "none":
            logger.warning("Auto-drive rejected — no base navigation active")
            self._auto_drive = False
            self._broadcast_status()
            return
        self._auto_drive = enabled
        if enabled:
            if self._mode != "navigate":
                self._mode = "navigate"      # force navigate mode
            # Auto-enable analysis if not already running
            if not self._enabled:
                self.set_enabled(True)
        logger.info("Auto-drive → %s  (base_nav=%s)",
                    "ON" if enabled else "OFF", self._base_nav_mode)
        self._broadcast_status()

    def set_base_nav(self, mode: str) -> None:
        """Update the base navigation mode.

        Called by app.py when:
        - Manual forward is pressed  → 'manual'
        - Waypoint nav is started    → 'waypoint'
        - Navigation/driving stops   → 'none'

        When base nav goes to 'none', auto-drive auto-disables.
        """
        prev = self._base_nav_mode
        self._base_nav_mode = mode if mode in ("manual", "waypoint") else "none"
        logger.info("Base nav: %s → %s", prev, self._base_nav_mode)

        # Auto-disable auto-drive when base navigation stops
        if self._base_nav_mode == "none" and self._auto_drive:
            logger.info("Auto-drive auto-disabled — base navigation stopped")
            self._auto_drive = False
        self._broadcast_status()

    # ─── Navigation decision parser ───────────────────────────────────
    @staticmethod
    def _parse_nav_decision(text: str) -> Dict[str, str]:
        """
        Parse the structured navigate response into
        {safety, direction, reason}.

        Expected format:
            SAFETY: SAFE
            DIRECTION: FORWARD
            REASON: Clear path ahead
        """
        safety = "DANGER"       # default safe-side: stop if parse fails
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
        """Push an AI_DRIVE command to the Pi via the instant queue.

        Gating rules:
        - Only send if base navigation is active (manual or waypoint).
        - During WAYPOINT nav: only send CAUTION/DANGER commands.
          If SAFE+FORWARD, let NavController handle steering — don't
          interfere with its heading corrections.
        - During MANUAL nav: send all commands (AI is the supervisor).
        """
        if not self._drive_command_fn:
            return
        if self._base_nav_mode == "none":
            return  # no base nav → no commands

        # During waypoint nav, NavController handles forward driving.
        # Only intervene when there's an obstacle (CAUTION/DANGER).
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
        """Kick off model download + load in a background thread."""
        if self._status in ("loading", "ready", "unavailable"):
            return
        self._status = "loading"
        self._error_msg = ""
        threading.Thread(
            target=self._load_model_sync, daemon=True, name="moondream-loader"
        ).start()
        logger.info("Moondream model load started (background)…")

    def _load_model_sync(self) -> None:
        """Synchronous model load — runs in daemon thread."""
        try:
            logger.info(
                "Loading %s  revision=%s …", self._model_id, self._revision
            )

            # Pick device
            if self._preferred_device == "auto":
                if torch.cuda.is_available():
                    self._device = "cuda"
                    self._dtype = torch.float16
                elif hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
                    self._device = "mps"
                    self._dtype = torch.float16
                else:
                    self._device = "cpu"
                    self._dtype = torch.float32
            else:
                self._device = self._preferred_device
                self._dtype = (
                    torch.float16 if self._device != "cpu" else torch.float32
                )

            logger.info("Device: %s   dtype: %s", self._device, self._dtype)

            model = AutoModelForCausalLM.from_pretrained(
                self._model_id,
                revision=self._revision,
                trust_remote_code=True,
                torch_dtype=self._dtype,
                low_cpu_mem_usage=True,
                device_map={"": self._device} if self._device != "cpu" else None,
            )
            if self._device == "cpu":
                model = model.to(self._device)
            model.eval()

            with self._lock:
                self._model = model
                self._status = "ready"

            logger.info("✓ Moondream ready on %s", self._device)
            self._broadcast_status()

        except Exception as exc:
            with self._lock:
                self._status = "error"
                self._error_msg = str(exc)
            logger.error("Moondream load failed: %s", exc)
            self._broadcast_status()

    def unload_model(self) -> None:
        """Free GPU / RAM."""
        self.set_enabled(False)
        with self._lock:
            if self._model is not None:
                del self._model
                self._model = None
                if _TORCH_AVAILABLE and torch.cuda.is_available():
                    torch.cuda.empty_cache()
            self._status = "not_loaded"
            self._error_msg = ""
        logger.info("Moondream model unloaded")
        self._broadcast_status()

    # ─── Analysis control ─────────────────────────────────────────────
    def set_enabled(self, enabled: bool) -> None:
        """Toggle the continuous analysis loop."""
        if enabled and self._status != "ready":
            if self._status == "not_loaded":
                self.load_model()          # auto-load on first enable
            return                          # will enable once ready

        self._enabled = enabled

        if enabled:
            if self._analysis_thread is None or not self._analysis_thread.is_alive():
                self._stop_event.clear()
                self._analysis_thread = threading.Thread(
                    target=self._analysis_loop, daemon=True,
                    name="moondream-analysis",
                )
                self._analysis_thread.start()
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

    # ─── Core inference ───────────────────────────────────────────────
    def _run_analysis(self, frame_bytes: bytes) -> Optional[Dict]:
        """Run one analysis on a JPEG frame.  Called from bg thread."""
        if self._model is None or not frame_bytes:
            return None
        try:
            image = Image.open(io.BytesIO(frame_bytes)).convert("RGB")
            t0 = time.time()

            result: Dict[str, Any] = {
                "timestamp": time.time(),
                "mode": self._mode,
                "count": self._analysis_count,
            }

            if self._mode == "detect":
                detections = self._model.detect(image, self._detect_target)
                objects: List[Dict] = detections.get("objects", [])
                result["detections"] = objects
                result["detect_target"] = self._detect_target
                result["num_objects"] = len(objects)
                result["response"] = (
                    f"Detected {len(objects)} '{self._detect_target}' object(s)"
                )

            elif self._mode == "navigate":
                prompt = PROMPTS["navigate"]
                answer = self._model.query(image, prompt)
                raw = answer.get("answer", "")
                result["response"] = raw
                result["prompt"] = prompt

                # Parse structured nav decision
                nav = self._parse_nav_decision(raw)
                result["nav_decision"] = nav
                self._last_nav = nav

                # Auto-drive: send command to Pi if enabled
                if self._auto_drive:
                    self._send_drive_command(nav)
                    result["drive_sent"] = True
                    result["drive_count"] = self._drive_count

            elif self._mode == "custom" and self._custom_prompt:
                answer = self._model.query(image, self._custom_prompt)
                result["response"] = answer.get("answer", "")
                result["prompt"] = self._custom_prompt

            else:
                prompt = PROMPTS.get(self._mode, PROMPTS["scene"])
                answer = self._model.query(image, prompt)
                result["response"] = answer.get("answer", "")
                result["prompt"] = prompt

            result["inference_time"] = round(time.time() - t0, 2)
            logger.debug(
                "AI [%s] %.2fs — %s",
                self._mode, result["inference_time"],
                (result.get("response") or "")[:100],
            )
            return result

        except Exception as exc:
            logger.error("Inference error: %s", exc)
            return None

    # ─── One-shot helpers (manual from API) ───────────────────────────
    def query_once(self, frame_bytes: bytes, prompt: str) -> Optional[Dict]:
        """Blocking single query — use from a Flask route."""
        if self._model is None or not frame_bytes:
            return None
        try:
            image = Image.open(io.BytesIO(frame_bytes)).convert("RGB")
            t0 = time.time()
            answer = self._model.query(image, prompt)
            return {
                "timestamp": time.time(),
                "mode": "manual",
                "prompt": prompt,
                "response": answer.get("answer", ""),
                "inference_time": round(time.time() - t0, 2),
                "count": self._analysis_count,
            }
        except Exception as exc:
            logger.error("Manual query error: %s", exc)
            return None

    def detect_once(self, frame_bytes: bytes, target: str) -> Optional[Dict]:
        """Blocking single detection — use from a Flask route."""
        if self._model is None or not frame_bytes:
            return None
        try:
            image = Image.open(io.BytesIO(frame_bytes)).convert("RGB")
            t0 = time.time()
            detections = self._model.detect(image, target)
            objects = detections.get("objects", [])
            return {
                "timestamp": time.time(),
                "mode": "detect",
                "detect_target": target,
                "detections": objects,
                "num_objects": len(objects),
                "response": f"Detected {len(objects)} '{target}' object(s)",
                "inference_time": round(time.time() - t0, 2),
                "count": self._analysis_count,
            }
        except Exception as exc:
            logger.error("Detection error: %s", exc)
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
            "transformers_available": _TRANSFORMERS_AVAILABLE,
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
            except Exception:
                pass
