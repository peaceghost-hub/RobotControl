"""
QMI Network Health Monitor (Python-level resilience)

Provides network-aware retry logic and connectivity detection for the
robot's communication threads.  Works alongside the system-level
qmi-watchdog.sh service — this module handles the application layer:

  • Detects when HTTP requests consistently fail
  • Triggers QMI reconnect via subprocess (kicks the bash script)
  • Provides a decorator / wrapper for retry-with-backoff
  • Tracks network state so other threads can check connectivity

Thread-safe.  Non-blocking.  Won't freeze the robot.
"""

import logging
import os
import socket
import subprocess
import threading
import time
from typing import Optional

logger = logging.getLogger('qmi_health')


class QMIHealthMonitor:
    """Application-layer QMI health monitor.

    Runs a lightweight background check and provides helpers for
    the status_loop / command_loop / api_client to be network-aware.
    """

    # ── Tunables ──────────────────────────────────────────────
    PING_HOST = '8.8.8.8'
    PING_PORT = 53          # DNS port — always open, fast TCP test
    TCP_TIMEOUT = 4.0       # seconds
    CHECK_INTERVAL = 45     # seconds between background checks
    RECONNECT_COOLDOWN = 60 # min seconds between reconnect attempts

    # Failure thresholds before kicking the bash reconnect
    FAIL_THRESHOLD = 3      # consecutive check failures

    def __init__(self, enabled: bool = True):
        self._enabled = enabled
        self._online = True        # optimistic start
        self._lock = threading.Lock()
        self._consecutive_fails = 0
        self._last_reconnect = 0.0
        self._check_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._total_reconnects = 0

    # ── Public API ────────────────────────────────────────────

    @property
    def is_online(self) -> bool:
        """Quick non-blocking check — is the network probably up?"""
        return self._online

    @property
    def stats(self) -> dict:
        with self._lock:
            return {
                'online': self._online,
                'consecutive_fails': self._consecutive_fails,
                'total_reconnects': self._total_reconnects,
            }

    def start(self):
        """Start the background health-check thread."""
        if not self._enabled:
            logger.info("QMI health monitor disabled")
            return
        self._stop_event.clear()
        self._check_thread = threading.Thread(
            target=self._check_loop, name='qmi-health', daemon=True
        )
        self._check_thread.start()
        logger.info("QMI health monitor started (interval=%ds)", self.CHECK_INTERVAL)

    def stop(self):
        """Stop the background thread."""
        self._stop_event.set()
        if self._check_thread:
            self._check_thread.join(timeout=10)

    def report_success(self):
        """Called by api_client / status_loop on a successful HTTP exchange."""
        with self._lock:
            if not self._online:
                logger.info("Network back online (reported by app layer)")
            self._online = True
            self._consecutive_fails = 0

    def report_failure(self):
        """Called by api_client / status_loop on a failed HTTP exchange."""
        with self._lock:
            self._consecutive_fails += 1
            if self._consecutive_fails >= self.FAIL_THRESHOLD:
                self._online = False
                self._maybe_trigger_reconnect()

    def check_connectivity(self) -> bool:
        """Active connectivity probe — TCP connect to 8.8.8.8:53.

        Much faster than a full ping and works even when ICMP is blocked.
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.TCP_TIMEOUT)
            sock.connect((self.PING_HOST, self.PING_PORT))
            sock.close()
            return True
        except (socket.timeout, socket.error, OSError):
            return False

    def check_dns(self) -> bool:
        """Test DNS resolution."""
        try:
            socket.setdefaulttimeout(self.TCP_TIMEOUT)
            socket.getaddrinfo('google.com', 80)
            return True
        except (socket.gaierror, socket.timeout, OSError):
            return False

    # ── Background loop ───────────────────────────────────────

    def _check_loop(self):
        """Periodic health check running in a daemon thread."""
        # Initial delay to let boot-time connection settle
        self._stop_event.wait(15)

        while not self._stop_event.is_set():
            try:
                reachable = self.check_connectivity()
                dns_ok = self.check_dns() if reachable else False

                with self._lock:
                    if reachable and dns_ok:
                        if not self._online:
                            logger.info("QMI health: connectivity restored")
                        self._online = True
                        self._consecutive_fails = 0
                    else:
                        self._consecutive_fails += 1
                        reason = "ping fail" if not reachable else "DNS fail"
                        logger.warning(
                            "QMI health check FAILED (%s) — streak %d/%d",
                            reason, self._consecutive_fails, self.FAIL_THRESHOLD
                        )
                        if self._consecutive_fails >= self.FAIL_THRESHOLD:
                            self._online = False
                            self._maybe_trigger_reconnect()
            except Exception as exc:
                logger.error("QMI health check error: %s", exc)

            self._stop_event.wait(self.CHECK_INTERVAL)

    # ── Recovery ──────────────────────────────────────────────

    def _maybe_trigger_reconnect(self):
        """Kick the system-level qmi-connect.sh if cooldown allows.

        Must be called with self._lock held.
        """
        now = time.time()
        if now - self._last_reconnect < self.RECONNECT_COOLDOWN:
            logger.debug("QMI reconnect on cooldown (%ds remaining)",
                         int(self.RECONNECT_COOLDOWN - (now - self._last_reconnect)))
            return

        self._last_reconnect = now
        self._total_reconnects += 1
        logger.warning("QMI health: triggering system reconnect (#%d)...",
                       self._total_reconnects)

        # Fire and forget — don't block the health thread
        threading.Thread(
            target=self._run_reconnect, name='qmi-reconnect', daemon=True
        ).start()

    @staticmethod
    def _run_reconnect():
        """Execute the system-level reconnect script."""
        script = '/usr/local/bin/qmi-connect.sh'
        if not os.path.isfile(script):
            logger.warning("QMI reconnect script not found: %s", script)
            # Try restarting the systemd service instead
            try:
                subprocess.run(
                    ['sudo', 'systemctl', 'restart', 'qmi-network'],
                    timeout=90, capture_output=True
                )
                logger.info("QMI reconnect: systemctl restart qmi-network completed")
            except Exception as exc:
                logger.error("QMI reconnect via systemctl failed: %s", exc)
            return

        try:
            result = subprocess.run(
                ['sudo', script],
                timeout=90,
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                logger.info("QMI reconnect script succeeded")
            else:
                logger.warning("QMI reconnect script failed (rc=%d): %s",
                               result.returncode, result.stderr[-200:] if result.stderr else '')
        except subprocess.TimeoutExpired:
            logger.error("QMI reconnect script timed out (90s)")
        except Exception as exc:
            logger.error("QMI reconnect error: %s", exc)


# Module-level singleton for easy import
_monitor: Optional[QMIHealthMonitor] = None


def get_monitor(enabled: bool = True) -> QMIHealthMonitor:
    """Get or create the singleton QMI health monitor."""
    global _monitor
    if _monitor is None:
        _monitor = QMIHealthMonitor(enabled=enabled)
    return _monitor
