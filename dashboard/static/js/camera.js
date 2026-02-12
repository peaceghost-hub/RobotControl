/**
 * Camera module: MJPEG stream with direct Pi stream support.
 *
 * Priority:
 *   1. Direct Pi MJPEG stream (lowest latency - Pi -> Browser)
 *   2. Dashboard relay MJPEG at /camera/mjpeg (Pi -> Dashboard -> Browser)
 *
 * The direct URL can be set via dashboard_config.camera.streamUrl or
 * CAMERA_STREAM_URL env var.  When the direct stream errors out (e.g.
 * Pi not reachable from browser network) we auto-fallback to the relay.
 */

function initCamera() {
  const cameraFeed = document.getElementById('camera-feed');
  const status = document.getElementById('camera-status');
  const noCamera = document.getElementById('no-camera');

  if (!cameraFeed) {
    console.error('camera-feed element not found; cannot start MJPEG stream');
    if (status) {
      status.textContent = 'No camera element';
      status.style.background = 'rgba(239, 68, 68, 0.3)';
    }
    return;
  }

  // Try to read direct stream URL from dashboard config
  var camCfg = (window.DASHBOARD_CONFIG && window.DASHBOARD_CONFIG.camera) || {};
  var directUrl = camCfg.streamUrl || '';
  var relayUrl = '/camera/mjpeg';

  function setStream(url, label) {
    cameraFeed.src = url;
    cameraFeed.classList.add('active');
    if (noCamera) noCamera.style.display = 'none';
    if (status) {
      status.textContent = label;
      status.style.background = 'rgba(16, 185, 129, 0.3)';
    }
  }

  function fallbackToRelay() {
    console.warn('Direct Pi stream failed; falling back to dashboard relay');
    cameraFeed.onerror = null;
    setStream(relayUrl, 'Live (MJPEG relay)');
  }

  if (directUrl) {
    console.log('Attempting direct Pi MJPEG stream:', directUrl);
    cameraFeed.onerror = fallbackToRelay;
    setStream(directUrl, 'Live (Direct Pi) — connecting…');

    // Timeout: if no frame arrives within 5 seconds, fall back to relay.
    // On GSM the direct Pi stream is often unreachable from the browser,
    // so we can't wait forever.
    var streamTimeout = setTimeout(function() {
      // Check if image actually loaded (naturalWidth > 0 means pixels arrived)
      if (!cameraFeed.naturalWidth || cameraFeed.naturalWidth < 2) {
        console.warn('Direct Pi stream timed out after 5s; falling back to relay');
        cameraFeed.onerror = null;
        setStream(relayUrl, 'Live (MJPEG relay)');
      }
    }, 5000);

    // If a frame does arrive, cancel the timeout and update the label
    cameraFeed.onload = function() {
      clearTimeout(streamTimeout);
      cameraFeed.onload = null;
      if (status) {
        status.textContent = 'Live (Direct Pi)';
      }
    };
  } else {
    setStream(relayUrl, 'Live (MJPEG)');
  }
}

document.addEventListener('DOMContentLoaded', function() {
  console.log('camera.js loaded; initializing MJPEG');
  initCamera();
});
