"""
Moondream AI Vision Engine — Cloud + Local Fallback
====================================================

Dual-backend AI vision for robot navigation:

1. **Cloud API (primary)** — Moondream Cloud via ``moondream`` SDK.
   Fast (~1-3 s), runs Moondream 3 (9B MoE), free tier $5/month.
2. **Local model (fallback)** — Native torch Moondream 2 (2B dense).
   Slow (~90-120 s on CPU), but works offline / no-signal situations.

The engine automatically falls back to local when the cloud is
unreachable, and switches back when connectivity is restored.

CRITICAL DESIGN PRINCIPLES
---------------------------
• **ADVISORY ONLY** — AI auto-drive sends suggestions into the command
  queue.  Manual arrows and waypoint navigation are the primary controls
  and always work regardless of AI state.
• **NO FREEZING** — every operation is non-blocking.  Cloud calls have a
  timeout; local inference runs in a background thread.
• **NO BREAKING** — all existing nav (manual, waypoint) is unaffected by
  AI being on, off, loading, or erroring.
"""

from __future__ import annotations

import io
import os
import time
import logging
import threading
from typing import Optional, Dict, Any, Callable, List

logger = logging.getLogger("ai_vision")

# ── Load .env file (so MOONDREAM_API_KEY persists across restarts) ───
def _load_dotenv() -> None:
    """Read dashboard/.env into os.environ (no dependency needed)."""
    env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
    if not os.path.isfile(env_path):
        return
    try:
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" in line:
                    key, _, value = line.partition("=")
                    key = key.strip()
                    value = value.strip().strip("'\"")
                    if key and key not in os.environ:  # don't override explicit exports
                        os.environ[key] = value
                        logger.debug(".env → %s=***", key)
    except Exception as exc:
        logger.warning("Failed to read .env: %s", exc)

_load_dotenv()

# ── Lazy heavy imports ───────────────────────────────────────────────
_TORCH_AVAILABLE = False
_PIL_AVAILABLE = False
_CLOUD_SDK_AVAILABLE = False

try:
    from PIL import Image
    _PIL_AVAILABLE = True
except ImportError:
    Image = None  # type: ignore[misc,assignment]

try:
    import moondream as md
    _CLOUD_SDK_AVAILABLE = True
except ImportError:
    md = None  # type: ignore[assignment]

try:
    import torch
    _TORCH_AVAILABLE = True
    torch.set_num_threads(os.cpu_count() or 4)
    torch._dynamo.config.suppress_errors = True  # type: ignore[attr-defined]
except ImportError:
    torch = None  # type: ignore[assignment]

# ── Robot physical parameters ────────────────────────────────────────
ROBOT_HEIGHT_CM = 20
ROBOT_WIDTH_CM = 20
WHEEL_RADIUS_CM = 2
ACTION_DISTANCE_CM = 20

_VISION_CROP_PX = 378  # max image edge for local model

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

# Cloud API key — set via env var or .env file
_CLOUD_API_KEY = os.getenv("MOONDREAM_API_KEY", "")

# Cloud request timeout (seconds) — fail fast, fall back to local
_CLOUD_TIMEOUT = float(os.getenv("MOONDREAM_CLOUD_TIMEOUT", "30"))

# Local weights path
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
    ai_paused = False
    last_nav = None
    drive_count = 0
    base_nav_mode = "none"

    _enabled = False
    _status = "unavailable"
    _auto_drive = False
    _ai_paused = False

    def load_model(self) -> None: ...
    def unload_model(self) -> None: ...
    def set_enabled(self, _e: bool) -> None: ...
    def set_mode(self, *_a: Any) -> None: ...
    def set_interval(self, _s: float) -> None: ...
    def set_auto_drive(self, _e: bool) -> None: ...
    def set_ai_paused(self, _p: bool) -> None: ...
    def set_base_nav(self, _m: str) -> None: ...
    def obstacle_trigger(self, _f: bytes, _d: int) -> None: ...
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
            "error": "AI Vision dependencies not installed (pip install moondream pillow)",
            "last_result": None,
            "torch_available": _TORCH_AVAILABLE,
            "transformers_available": False,
            "auto_drive": False,
            "ai_paused": False,
            "last_nav": None,
            "drive_count": 0,
            "base_nav_mode": "none",
            "backend": "none",
            "robot_params": {
                "height_cm": ROBOT_HEIGHT_CM,
                "width_cm": ROBOT_WIDTH_CM,
                "wheel_radius_cm": WHEEL_RADIUS_CM,
                "action_distance_cm": ACTION_DISTANCE_CM,
            },
        }


NullVision = _NullVision  # public alias


# ======================================================================
#  Main engine — Cloud primary, local fallback
# ======================================================================
class MoondreamVision:
    """Non-blocking Moondream vision engine with dual backend.

    Primary: Moondream Cloud API (~1-3 s response, Moondream 3)
    Fallback: Local native torch model (~90-120 s on CPU, Moondream 2)

    The AI auto-drive is ADVISORY ONLY.  Manual control and waypoint
    navigation always work regardless of AI state.
    """

    # ── Class-level sampling presets ──────────────────────────────────
    _NAV_SETTINGS: dict = {"temperature": 0, "max_tokens": 100}
    _SCENE_SETTINGS: dict = {"temperature": 0, "max_tokens": 256}
    _CUSTOM_SETTINGS: dict = {"temperature": 0.5, "max_tokens": 512}

    def __init__(
        self,
        weights_path: Optional[str] = None,
        device: str = "auto",
    ):
        self._weights_path = weights_path or _DEFAULT_WEIGHTS_PATH
        self._model_id = DEFAULT_MODEL_ID
        self._preferred_device = device

        # ── Backend state ─────────────────────────────────────────────
        self._cloud_client: Any = None       # moondream SDK client
        self._local_model: Any = None        # native torch MoondreamModel
        self._local_device: Optional[str] = None
        self._local_dtype: Any = None

        # Which backend is active: "cloud", "local", or "none"
        self._active_backend = "none"
        self._cloud_available = False
        self._local_available = False
        self._cloud_consecutive_fails = 0
        self._cloud_backoff_until = 0.0  # timestamp

        self._status = "not_loaded"
        self._error_msg = ""
        self._lock = threading.Lock()

        # ── Analysis state ────────────────────────────────────────────
        self._enabled = False
        self._interval = DEFAULT_INTERVAL
        self._mode = "scene"
        self._detect_target = "obstacle"
        self._custom_prompt = ""
        self._last_result: Optional[Dict] = None
        self._analysis_count = 0
        self._desired_enabled = False

        # Background thread
        self._analysis_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # ── Auto-drive (ADVISORY — never blocks primary nav) ──────────
        self._auto_drive = False
        self._ai_paused = False        # quick-toggle: pause drive commands
        self._base_nav_mode = "none"
        self._last_nav: Optional[Dict[str, str]] = None
        self._drive_count = 0

        # ── Obstacle-triggered analysis (event-driven when auto-drive ON) ──
        # When auto_drive is enabled the analysis loop does NOT poll on a
        # timer.  Instead it waits on this Event, which is set by
        # ``obstacle_trigger()`` when the Pi reports OBSTACLE_DETECTED.
        self._obstacle_event = threading.Event()
        self._obstacle_frame: Optional[bytes] = None
        self._obstacle_distance: int = -1
        self._obstacle_lock = threading.Lock()  # guards frame/distance

        # ── Injected by app.py ────────────────────────────────────────
        self._frame_provider: Optional[Callable[[], Optional[bytes]]] = None
        self._emit_fn: Optional[Callable[[str, Dict], None]] = None
        self._drive_command_fn: Optional[Callable[[Dict], None]] = None

        # ── Try to initialise Cloud immediately (fast, no blocking) ───
        self._init_cloud()

        logger.info(
            "MoondreamVision init — cloud_sdk=%s  api_key=%s  torch=%s  weights=%s",
            _CLOUD_SDK_AVAILABLE,
            "set" if _CLOUD_API_KEY else "NOT SET",
            _TORCH_AVAILABLE,
            "found" if os.path.exists(self._weights_path) else "missing",
        )

    # ══════════════════════════════════════════════════════════════════
    #  BACKEND INITIALISATION
    # ══════════════════════════════════════════════════════════════════

    def _init_cloud(self) -> None:
        """Try to create the Cloud SDK client.  Instant, non-blocking."""
        if not _CLOUD_SDK_AVAILABLE:
            logger.info("Cloud SDK not installed (pip install moondream)")
            return
        if not _CLOUD_API_KEY:
            logger.info("MOONDREAM_API_KEY not set — cloud disabled")
            return
        try:
            self._cloud_client = md.vl(api_key=_CLOUD_API_KEY)
            self._cloud_available = True
            self._active_backend = "cloud"
            self._status = "ready"
            logger.info("✓ Moondream Cloud API ready")
        except Exception as exc:
            logger.warning("Cloud SDK init failed: %s", exc)
            self._cloud_available = False

    def _init_local_sync(self) -> None:
        """Load the local 2B model — runs in a daemon thread."""
        if not _TORCH_AVAILABLE:
            self._error_msg = "torch not installed — local fallback unavailable"
            logger.warning(self._error_msg)
            return
        if not os.path.exists(self._weights_path):
            self._error_msg = f"Weights not found: {self._weights_path}"
            logger.warning(self._error_msg)
            return

        try:
            import sys
            dashboard_dir = os.path.dirname(os.path.abspath(__file__))
            if dashboard_dir not in sys.path:
                sys.path.insert(0, dashboard_dir)

            from md_reference.config import MoondreamConfig, TextConfig
            from md_reference.moondream import MoondreamModel
            from md_reference.weights import load_weights_into_model

            logger.info("Loading local model from %s …", self._weights_path)

            # Pick device + dtype
            if self._preferred_device == "auto":
                if torch.cuda.is_available():
                    self._local_device = "cuda"
                    self._local_dtype = torch.float16
                elif hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
                    self._local_device = "mps"
                    self._local_dtype = torch.float16
                else:
                    self._local_device = "cpu"
                    self._local_dtype = torch.float32
            else:
                self._local_device = self._preferred_device
                self._local_dtype = (
                    torch.float16 if self._local_device != "cpu" else torch.float32
                )

            t0 = time.time()
            text_cfg = TextConfig(moe=None)
            config = MoondreamConfig(text=text_cfg)
            model = MoondreamModel(config, dtype=self._local_dtype, setup_caches=True)
            load_weights_into_model(self._weights_path, model)
            model = model.to(device=self._local_device, dtype=self._local_dtype)
            model.eval()
            if self._local_device == "cpu":
                model.use_flex_decoding = False
            model._setup_caches()

            with self._lock:
                self._local_model = model
                self._local_available = True
            logger.info(
                "✓ Local model ready (%s, %s) in %.1fs",
                self._local_device, self._local_dtype, time.time() - t0,
            )

            # If cloud isn't available, local becomes active
            if not self._cloud_available:
                self._active_backend = "local"
                with self._lock:
                    self._status = "ready"
                if self._desired_enabled:
                    self.set_enabled(True)
                else:
                    self._broadcast_status()

        except Exception as exc:
            self._error_msg = f"Local model load failed: {exc}"
            logger.error(self._error_msg, exc_info=True)
            if not self._cloud_available:
                with self._lock:
                    self._status = "error"
                self._broadcast_status()

    # ══════════════════════════════════════════════════════════════════
    #  PROPERTIES
    # ══════════════════════════════════════════════════════════════════

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

    # ══════════════════════════════════════════════════════════════════
    #  AUTO-DRIVE (ADVISORY — never blocks primary navigation)
    # ══════════════════════════════════════════════════════════════════

    def set_auto_drive(self, enabled: bool) -> None:
        self._auto_drive = enabled
        if enabled:
            # Switch display mode to navigate so the UI shows nav results,
            # but do NOT start navigation or enable analysis automatically.
            # The user starts movement themselves (arrows or nav start).
            if self._mode != "navigate":
                self._mode = "navigate"
            # Clear pause when toggling on
            self._ai_paused = False
        logger.info(
            "Auto-drive → %s  (base_nav=%s, backend=%s)",
            "ON" if enabled else "OFF",
            self._base_nav_mode,
            self._active_backend,
        )
        self._broadcast_status()

    def set_ai_paused(self, paused: bool) -> None:
        """Quick-toggle to pause/resume AI drive commands.
        When paused, AI still analyses for display but never sends AI_DRIVE."""
        self._ai_paused = paused
        logger.info("AI drive paused → %s", paused)
        self._broadcast_status()

    @property
    def ai_paused(self) -> bool:
        return self._ai_paused

    def set_base_nav(self, mode: str) -> None:
        prev = self._base_nav_mode
        self._base_nav_mode = mode if mode in ("manual", "waypoint") else "none"
        if prev != self._base_nav_mode:
            logger.info("Base nav: %s → %s", prev, self._base_nav_mode)
        self._broadcast_status()

    def obstacle_trigger(self, frame_bytes: Optional[bytes] = None, distance_cm: int = -1) -> None:
        """Called by app.py when OBSTACLE_DETECTED arrives and AI is loaded.

        Stashes the frame + distance and signals the analysis thread to run
        an immediate navigate-mode analysis instead of waiting for the next
        timer tick.  If ``frame_bytes`` is None, the analysis loop will grab
        the latest frame via ``_frame_provider`` as usual.
        """
        if not self._enabled:
            return
        with self._obstacle_lock:
            self._obstacle_frame = frame_bytes
            self._obstacle_distance = distance_cm
        self._obstacle_event.set()
        logger.info("Obstacle trigger: dist=%dcm — AI analysis queued", distance_cm)

    # ══════════════════════════════════════════════════════════════════
    #  NAVIGATION DECISION PARSER & DRIVE COMMAND
    # ══════════════════════════════════════════════════════════════════

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
        """Send obstacle avoidance advice to the Pi.

        Only sends when auto-drive is ON and not paused.
        The Pi decides what to do with it:
        - NavController active → feeds to NavController
        - Manual driving → executes directly as motor command
        """
        if not self._drive_command_fn:
            return
        if not self._auto_drive or self._ai_paused:
            logger.debug("AI drive command suppressed (auto=%s, paused=%s)",
                         self._auto_drive, self._ai_paused)
            return

        try:
            self._drive_command_fn({
                "command": "AI_DRIVE",
                "payload": {
                    "direction": nav["direction"],
                    "safety": nav["safety"],
                    "reason": nav["reason"],
                    "backend": self._active_backend,
                },
            })
            self._drive_count += 1
            logger.info(
                "AI_DRIVE → %s (safety=%s, via=%s) — %s",
                nav["direction"], nav["safety"],
                self._active_backend,
                nav["reason"],
            )
        except Exception as exc:
            logger.error("Failed to send AI_DRIVE: %s", exc)

    # ══════════════════════════════════════════════════════════════════
    #  MODEL LIFECYCLE
    # ══════════════════════════════════════════════════════════════════

    def load_model(self) -> None:
        """Non-blocking model init.  Cloud is instant; local loads in bg."""
        if self._status in ("loading", "ready"):
            return

        # Cloud may already be ready from __init__
        if self._cloud_available:
            self._active_backend = "cloud"
            self._status = "ready"
            self._broadcast_status()
            # Also start loading local in background as offline fallback
            if not self._local_available and _TORCH_AVAILABLE:
                threading.Thread(
                    target=self._init_local_sync, daemon=True,
                    name="moondream-local-loader",
                ).start()
            if self._desired_enabled:
                self.set_enabled(True)
            return

        # No cloud — try local
        if _TORCH_AVAILABLE and os.path.exists(self._weights_path):
            self._status = "loading"
            self._error_msg = ""
            self._broadcast_status()
            threading.Thread(
                target=self._init_local_sync, daemon=True,
                name="moondream-local-loader",
            ).start()
            logger.info("Local model loading (background)…")
            return

        # Nothing available
        self._status = "error"
        self._error_msg = (
            "No backend available. Set MOONDREAM_API_KEY for cloud, "
            "or install torch + download weights for local."
        )
        self._broadcast_status()

    def unload_model(self) -> None:
        self.set_enabled(False)
        with self._lock:
            if self._local_model is not None:
                del self._local_model
                self._local_model = None
                self._local_available = False
                if _TORCH_AVAILABLE and torch.cuda.is_available():
                    torch.cuda.empty_cache()
            # Cloud client stays alive (stateless, no resources to free)
            if not self._cloud_available:
                self._status = "not_loaded"
            else:
                self._status = "ready"  # cloud still available
            self._error_msg = ""
        logger.info("Local model unloaded (cloud still %s)",
                    "available" if self._cloud_available else "unavailable")
        self._broadcast_status()

    # ══════════════════════════════════════════════════════════════════
    #  ANALYSIS CONTROL
    # ══════════════════════════════════════════════════════════════════

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
                "AI Vision ON  (%.1fs  mode=%s  backend=%s)",
                self._interval, self._mode, self._active_backend,
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
        self._interval = max(1.0, min(60.0, seconds))
        logger.info("AI interval → %.1fs", self._interval)

    def _start_analysis_thread(self) -> None:
        if self._analysis_thread is None or not self._analysis_thread.is_alive():
            self._stop_event.clear()
            self._analysis_thread = threading.Thread(
                target=self._analysis_loop, daemon=True,
                name="moondream-analysis",
            )
            self._analysis_thread.start()

    # ══════════════════════════════════════════════════════════════════
    #  BACKGROUND ANALYSIS LOOP
    # ══════════════════════════════════════════════════════════════════

    def _analysis_loop(self) -> None:
        logger.info("Analysis loop started (backend=%s)", self._active_backend)
        while not self._stop_event.is_set():
            if not self._enabled or self._status != "ready":
                self._stop_event.wait(0.5)
                continue

            # ── Wait for next trigger: obstacle event OR timer interval ──
            # Obstacle events take priority (instant, event-driven).
            # If no obstacle fires within _interval, run regular analysis.
            triggered = self._obstacle_event.wait(timeout=self._interval)
            if self._stop_event.is_set():
                break

            if triggered:
                self._obstacle_event.clear()

                # ── OBSTACLE-TRIGGERED ANALYSIS (navigate mode) ───────
                # Grab the stashed obstacle frame (or fall back to live frame)
                with self._obstacle_lock:
                    frame_bytes = self._obstacle_frame
                    obs_dist = self._obstacle_distance
                    self._obstacle_frame = None
                if frame_bytes is None:
                    frame_bytes = (
                        self._frame_provider() if self._frame_provider else None
                    )
                if frame_bytes is None:
                    continue

                # Broadcast "analysing" to the UI
                if self._emit_fn:
                    self._emit_fn("ai_vision_update", {
                        "mode": "navigate",
                        "response": f"Obstacle at {obs_dist}cm — AI analysing… ({self._active_backend})",
                        "inference_time": None,
                        "count": self._analysis_count,
                        "backend": self._active_backend,
                        "obstacle_triggered": True,
                    })

                # Force navigate mode for obstacle analysis so we get
                # a directional decision (FORWARD/LEFT/RIGHT/STOP).
                saved_mode = self._mode
                self._mode = "navigate"
                result = self._run_analysis(frame_bytes)
                self._mode = saved_mode  # restore original mode

                if result:
                    result["obstacle_triggered"] = True
                    result["obstacle_distance_cm"] = obs_dist

                    # Send drive command ONLY from obstacle-triggered path.
                    # Timer-based analysis never sends drive commands.
                    nav = result.get("nav_decision")
                    if nav:
                        self._send_drive_command(nav)
                        result["drive_sent"] = True
                        result["drive_count"] = self._drive_count

                    with self._lock:
                        self._last_result = result
                        self._analysis_count += 1
                    if self._emit_fn:
                        self._emit_fn("ai_vision_update", result)
                continue  # loop back — don't fall through to timer path

            # ── TIMER-BASED PATH (regular analysis) ─────────────────
            try:
                frame_bytes = (
                    self._frame_provider() if self._frame_provider else None
                )
                if frame_bytes is None:
                    self._stop_event.wait(1.0)
                    continue

                # Broadcast "analysing" to the UI
                if self._emit_fn:
                    self._emit_fn("ai_vision_update", {
                        "mode": self._mode,
                        "response": f"Analyzing… ({self._active_backend})",
                        "inference_time": None,
                        "count": self._analysis_count,
                        "backend": self._active_backend,
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

        logger.info("Analysis loop stopped")

    # ══════════════════════════════════════════════════════════════════
    #  CLOUD BACKEND CALLS
    # ══════════════════════════════════════════════════════════════════

    def _cloud_query(self, image: Any, prompt: str) -> Optional[Dict[str, Any]]:
        """Query via Cloud API.  Returns dict with 'answer' or None."""
        if not self._cloud_client:
            return None

        # Exponential backoff after consecutive failures
        if time.time() < self._cloud_backoff_until:
            return None

        try:
            result = self._cloud_client.query(image, prompt)
            self._cloud_consecutive_fails = 0
            self._cloud_backoff_until = 0.0
            if isinstance(result, dict):
                return result
            return {"answer": str(result)}
        except Exception as exc:
            self._cloud_consecutive_fails += 1
            # Back off: 5s, 10s, 20s, 40s, max 60s
            backoff = min(60, 5 * (2 ** (self._cloud_consecutive_fails - 1)))
            self._cloud_backoff_until = time.time() + backoff
            logger.warning(
                "Cloud query failed (%d consecutive, backoff %.0fs): %s",
                self._cloud_consecutive_fails, backoff, exc,
            )
            return None

    def _cloud_detect(self, image: Any, target: str) -> Optional[Dict[str, Any]]:
        """Detect via Cloud API."""
        if not self._cloud_client:
            return None
        if time.time() < self._cloud_backoff_until:
            return None
        try:
            result = self._cloud_client.detect(image, target)
            self._cloud_consecutive_fails = 0
            self._cloud_backoff_until = 0.0
            if isinstance(result, dict):
                return result
            return {"objects": []}
        except Exception as exc:
            self._cloud_consecutive_fails += 1
            backoff = min(60, 5 * (2 ** (self._cloud_consecutive_fails - 1)))
            self._cloud_backoff_until = time.time() + backoff
            logger.warning("Cloud detect failed: %s", exc)
            return None

    # ══════════════════════════════════════════════════════════════════
    #  LOCAL BACKEND CALLS
    # ══════════════════════════════════════════════════════════════════

    def _local_query(self, image: Any, prompt: str, settings: Dict) -> Optional[Dict]:
        """Query via local native torch model."""
        if self._local_model is None:
            return None
        try:
            with torch.inference_mode():
                result = self._local_model.query(
                    image, prompt, reasoning=False, stream=False,
                    settings=settings or {},
                )
            if isinstance(result, dict):
                return result
            return {"answer": str(result)}
        except Exception as exc:
            logger.error("Local query error: %s", exc)
            return None

    def _local_detect(self, image: Any, target: str) -> Optional[Dict]:
        """Detect via local model."""
        if self._local_model is None:
            return None
        try:
            with torch.inference_mode():
                result = self._local_model.detect(image, target)
            if isinstance(result, dict):
                return result
            return {"objects": []}
        except Exception as exc:
            logger.warning("Local detect failed, falling back to query: %s", exc)
            fb = self._local_query(
                image,
                f"Do you see any {target}? Count and describe briefly.",
                self._SCENE_SETTINGS,
            )
            if fb and "answer" in fb:
                return {"objects": [], "response": fb["answer"]}
            return None

    # ══════════════════════════════════════════════════════════════════
    #  UNIFIED INFERENCE (Cloud → Local fallback)
    # ══════════════════════════════════════════════════════════════════

    def _query(self, image: Any, prompt: str, settings: Dict) -> Dict[str, Any]:
        """Try cloud first, fall back to local.  Never blocks primary nav."""
        # Try cloud
        if self._cloud_available:
            result = self._cloud_query(image, prompt)
            if result is not None:
                self._active_backend = "cloud"
                result["_backend"] = "cloud"
                return result
            logger.info("Cloud unavailable, trying local fallback…")

        # Try local
        if self._local_available:
            result = self._local_query(image, prompt, settings)
            if result is not None:
                self._active_backend = "local"
                result["_backend"] = "local"
                return result

        return {"answer": "", "_backend": "none"}

    def _detect(self, image: Any, target: str) -> Dict[str, Any]:
        """Try cloud detect first, fall back to local."""
        if self._cloud_available:
            result = self._cloud_detect(image, target)
            if result is not None:
                self._active_backend = "cloud"
                result["_backend"] = "cloud"
                return result

        if self._local_available:
            result = self._local_detect(image, target)
            if result is not None:
                self._active_backend = "local"
                result["_backend"] = "local"
                return result

        return {"objects": [], "_backend": "none"}

    # ══════════════════════════════════════════════════════════════════
    #  IMAGE HELPERS
    # ══════════════════════════════════════════════════════════════════

    @staticmethod
    def _resize_for_model(image: Any) -> Any:
        """Shrink so longest edge ≤ _VISION_CROP_PX.  Saves local encode time."""
        w, h = image.size
        longest = max(w, h)
        if longest <= _VISION_CROP_PX:
            return image
        scale = _VISION_CROP_PX / longest
        return image.resize(
            (max(1, int(w * scale)), max(1, int(h * scale))),
            Image.LANCZOS,
        )

    @staticmethod
    def _bytes_to_image(frame_bytes: bytes) -> Any:
        """JPEG bytes → PIL RGB Image."""
        return Image.open(io.BytesIO(frame_bytes)).convert("RGB")

    # ══════════════════════════════════════════════════════════════════
    #  CORE ANALYSIS (called from bg thread)
    # ══════════════════════════════════════════════════════════════════

    def _run_analysis(self, frame_bytes: bytes) -> Optional[Dict]:
        """Run one analysis cycle.  Non-blocking for the Flask server."""
        if not frame_bytes:
            return None
        if not self._cloud_available and self._local_model is None:
            return None

        try:
            image = self._bytes_to_image(frame_bytes)
            # Resize only for local backend (cloud handles its own resizing)
            image_local = self._resize_for_model(image)
            image_for_query = image if self._cloud_available else image_local

            t0 = time.time()
            logger.info(
                "AI inference started (mode=%s, backend=%s, size=%s)",
                self._mode, self._active_backend, image_for_query.size,
            )

            result: Dict[str, Any] = {
                "timestamp": time.time(),
                "mode": self._mode,
                "count": self._analysis_count,
            }

            if self._mode == "detect":
                detections = self._detect(image_for_query, self._detect_target)
                objects: List[Dict] = (
                    detections.get("objects", [])
                    if isinstance(detections, dict) else []
                )
                result["detections"] = objects
                result["detect_target"] = self._detect_target
                result["num_objects"] = len(objects)
                resp = (
                    detections.get("response") or detections.get("text")
                    if isinstance(detections, dict) else None
                )
                result["response"] = resp or (
                    f"Detected {len(objects)} '{self._detect_target}' object(s)"
                )

            elif self._mode == "navigate":
                prompt = PROMPTS["navigate"]
                answer = self._query(image_for_query, prompt, self._NAV_SETTINGS)
                raw = answer.get("answer", "")
                result["response"] = raw
                result["prompt"] = prompt

                nav = self._parse_nav_decision(raw)
                result["nav_decision"] = nav
                self._last_nav = nav
                # NOTE: drive command is sent by the analysis loop caller,
                # NOT here — only obstacle-triggered analyses send commands.

            elif self._mode == "custom" and self._custom_prompt:
                answer = self._query(
                    image_for_query, self._custom_prompt, self._CUSTOM_SETTINGS
                )
                result["response"] = answer.get("answer", "")
                result["prompt"] = self._custom_prompt

            else:
                prompt = PROMPTS.get(self._mode, PROMPTS["scene"])
                answer = self._query(image_for_query, prompt, self._SCENE_SETTINGS)
                result["response"] = answer.get("answer", "")
                result["prompt"] = prompt

            result["inference_time"] = round(time.time() - t0, 2)
            result["backend"] = self._active_backend
            logger.info(
                "AI [%s] %.1fs via %s — %s",
                self._mode, result["inference_time"],
                self._active_backend,
                (result.get("response") or "")[:120],
            )
            return result

        except Exception as exc:
            logger.error("Inference error: %s", exc, exc_info=True)
            return None

    # ══════════════════════════════════════════════════════════════════
    #  ONE-SHOT HELPERS (manual queries from API routes)
    # ══════════════════════════════════════════════════════════════════

    def query_once(self, frame_bytes: bytes, prompt: str) -> Optional[Dict]:
        if not frame_bytes:
            return None
        if not self._cloud_available and self._local_model is None:
            return None
        try:
            image = self._bytes_to_image(frame_bytes)
            if not self._cloud_available:
                image = self._resize_for_model(image)
            t0 = time.time()
            answer = self._query(image, prompt, self._SCENE_SETTINGS)
            return {
                "timestamp": time.time(),
                "mode": "manual",
                "prompt": prompt,
                "response": answer.get("answer", ""),
                "inference_time": round(time.time() - t0, 2),
                "count": self._analysis_count,
                "backend": answer.get("_backend", self._active_backend),
            }
        except Exception as exc:
            logger.error("Manual query error: %s", exc, exc_info=True)
            return None

    def detect_once(self, frame_bytes: bytes, target: str) -> Optional[Dict]:
        if not frame_bytes:
            return None
        if not self._cloud_available and self._local_model is None:
            return None
        try:
            image = self._bytes_to_image(frame_bytes)
            if not self._cloud_available:
                image = self._resize_for_model(image)
            t0 = time.time()
            detections = self._detect(image, target)
            objects = (
                detections.get("objects", [])
                if isinstance(detections, dict) else []
            )
            resp = (
                detections.get("response")
                if isinstance(detections, dict) else None
            )
            return {
                "timestamp": time.time(),
                "mode": "detect",
                "detect_target": target,
                "detections": objects,
                "num_objects": len(objects),
                "response": resp or f"Detected {len(objects)} '{target}' object(s)",
                "inference_time": round(time.time() - t0, 2),
                "count": self._analysis_count,
                "backend": detections.get("_backend", self._active_backend),
            }
        except Exception as exc:
            logger.error("Detection error: %s", exc, exc_info=True)
            return None

    # ══════════════════════════════════════════════════════════════════
    #  STATUS
    # ══════════════════════════════════════════════════════════════════

    def get_status(self) -> Dict[str, Any]:
        return {
            "status": self._status,
            "enabled": self._enabled,
            "mode": self._mode,
            "interval": self._interval,
            "device": self._local_device or "cloud",
            "model_id": self._model_id,
            "analysis_count": self._analysis_count,
            "detect_target": self._detect_target,
            "custom_prompt": self._custom_prompt,
            "error": self._error_msg,
            "last_result": self._last_result,
            "torch_available": _TORCH_AVAILABLE,
            "transformers_available": True,  # compat flag
            "auto_drive": self._auto_drive,
            "ai_paused": self._ai_paused,
            "base_nav_mode": self._base_nav_mode,
            "last_nav": self._last_nav,
            "drive_count": self._drive_count,
            "backend": self._active_backend,
            "cloud_available": self._cloud_available,
            "local_available": self._local_available,
            "robot_params": {
                "height_cm": ROBOT_HEIGHT_CM,
                "width_cm": ROBOT_WIDTH_CM,
                "wheel_radius_cm": WHEEL_RADIUS_CM,
                "action_distance_cm": ACTION_DISTANCE_CM,
            },
        }

    def _broadcast_status(self) -> None:
        if self._emit_fn:
            try:
                self._emit_fn("ai_vision_status", self.get_status())
            except Exception as exc:
                logger.warning("_broadcast_status failed: %s", exc)
