/**
 * Camera Module - Handles live camera feed
 *
 * Improvements:
 * - Uses MJPEG <img src="/camera/mjpeg"> when the dashboard exposes MJPEG (relay mode).
 * - Subscribes to socket.io 'camera_frame' events as an optional real-time fallback (keeps legacy behavior).
 * - Polls /api/camera/latest as fallback when neither socket nor MJPEG available.
 */

let lastFrameTime = 0;
let frameCount = 0;
let fpsInterval;
let pollIntervalId = null;

/**
 * Initialize Camera Feed
 */
function initCamera() {
    // Start FPS counter
    fpsInterval = setInterval(updateFPS, 1000);

    // Setup snapshot button (may be missing in some layouts)
    const snapBtn = document.getElementById('snapshot-btn');
    if (snapBtn) {
        snapBtn.addEventListener('click', takeSnapshot);
    }

    applyCameraConfig();

    // If socket is available from main.js, subscribe; otherwise try to create one lazily
    try {
        if (window.socket && typeof window.socket.on === 'function') {
            window.socket.on('camera_frame', payload => {
                if (payload && payload.frame) {
                    updateCameraFeed(payload.frame, payload.timestamp);
                }
            });
        } else if (typeof io !== 'undefined') {
            // Create a lightweight socket only if none exists
            const baseUrl = window.CONFIG?.apiBaseUrl || '';
            try {
                // Connect with same-origin default if no baseUrl
                const socket = baseUrl ? io(baseUrl) : io();
                window.socket = socket;
                socket.on('camera_frame', payload => {
                    if (payload && payload.frame) {
                        updateCameraFeed(payload.frame, payload.timestamp);
                    }
                });
            } catch (e) {
                console.debug('Socket.IO not available or connection failed:', e);
            }
        }
    } catch (e) {
        console.debug('Error setting up socket camera listener:', e);
    }

    window.addLog && window.addLog('info', 'Camera module initialized');
}

function applyCameraConfig() {
    const config = window.CONFIG?.camera || {};
    const mode = config.streamMode || 'relay';
    const modeLabel = document.getElementById('camera-mode-label');
    const directHint = document.getElementById('camera-direct-hint');
    const directLink = document.getElementById('camera-direct-link');

    if (modeLabel) {
        modeLabel.textContent = mode === 'direct' ? 'Direct' : 'Relay';
    }

    const cameraFeed = document.getElementById('camera-feed');
    const noCamera = document.getElementById('no-camera');
    const status = document.getElementById('camera-status');

    if (mode === 'direct' && config.streamUrl) {
        if (directHint) directHint.style.display = 'block';
        if (directLink) {
            directLink.href = config.streamUrl;
            directLink.textContent = config.streamUrl;
        }

        // For direct streaming, set the camera img src directly
        if (cameraFeed) cameraFeed.src = config.streamUrl;
        if (cameraFeed) cameraFeed.classList.add('active');
        if (noCamera) noCamera.style.display = 'none';
        if (status) {
            status.textContent = 'Live';
            status.style.background = 'rgba(16, 185, 129, 0.3)';
        }
    } else {
        // Relay mode: prefer MJPEG served by dashboard if enabled
        if (directHint) directHint.style.display = 'none';
        if (directLink) directLink.href = '#';

        const mjpegEnabled = config.mjpegEnabled !== undefined ? config.mjpegEnabled : true;
        if (mjpegEnabled && cameraFeed) {
            // Point image source to MJPEG endpoint
            cameraFeed.src = (window.CONFIG?.apiBaseUrl || '') + '/camera/mjpeg';
            cameraFeed.classList.add('active');
            if (noCamera) noCamera.style.display = 'none';
            if (status) {
                status.textContent = 'Live (MJPEG)';
                status.style.background = 'rgba(16, 185, 129, 0.3)';
            }
            // No polling needed when MJPEG directly streams the images
            if (pollIntervalId) {
                clearInterval(pollIntervalId);
                pollIntervalId = null;
            }
            return;
        }

        // Fallback polling rate derived from configured frameRate
        const fps = Number(config.frameRate || config.fps || 2) || 2;
        // Polling interval: don't exceed server-side frame rate; lower bound 250ms
        const pollMs = Math.max(250, Math.round(1000 / Math.max(1, Math.min(fps, 25))));
        if (pollIntervalId) {
            clearInterval(pollIntervalId);
            pollIntervalId = null;
        }
        // Only poll if no socket is available (socket will deliver real-time frames)
        if (!window.socket && cameraFeed) {
            pollIntervalId = setInterval(requestCameraFrame, pollMs);
        }
    }
}

/**
 * Update Camera Feed
 */
function updateCameraFeed(frameBase64, timestamp) {
    const cameraFeed = document.getElementById('camera-feed');
    const noCamera = document.getElementById('no-camera');
    const cameraStatus = document.getElementById('camera-status');
    const cameraTimestamp = document.getElementById('camera-timestamp');

    if (!frameBase64) return;

    try {
        // Update image (base64 JPEG)
        if (cameraFeed) cameraFeed.src = `data:image/jpeg;base64,${frameBase64}`;
        if (cameraFeed) cameraFeed.classList.add('active');
        if (noCamera) noCamera.style.display = 'none';

        // Update status
        if (cameraStatus) {
            cameraStatus.textContent = 'Live';
            cameraStatus.style.background = 'rgba(16, 185, 129, 0.3)';
        }

        // Update timestamp
        if (timestamp && cameraTimestamp) {
            const time = new Date(timestamp);
            cameraTimestamp.textContent = time.toLocaleTimeString();
        }

        // Count frames for FPS
        frameCount++;
        lastFrameTime = Date.now();

    } catch (error) {
        console.error('Error updating camera feed:', error);
        window.addLog && window.addLog('error', 'Failed to update camera feed');
    }
}

/**
 * Update FPS Counter
 */
function updateFPS() {
    const fpsElement = document.getElementById('camera-fps');
    const cameraStatus = document.getElementById('camera-status');

    // Check if we're receiving frames
    const timeSinceLastFrame = Date.now() - lastFrameTime;

    if (timeSinceLastFrame > 3000) {
        // No frames for 3 seconds
        if (fpsElement) fpsElement.textContent = '0 FPS';
        if (cameraStatus) {
            cameraStatus.textContent = 'No Feed';
            cameraStatus.style.background = 'rgba(239, 68, 68, 0.3)';
        }

        // Hide feed, show placeholder
        const cameraFeed = document.getElementById('camera-feed');
        const noCamera = document.getElementById('no-camera');
        if (cameraFeed) cameraFeed.classList.remove('active');
        if (noCamera) noCamera.style.display = 'flex';
    } else {
        // Update FPS display
        if (fpsElement) fpsElement.textContent = `${frameCount} FPS`;
    }

    // Reset frame count
    frameCount = 0;
}

/**
 * Take Snapshot
 */
function takeSnapshot() {
    const cameraFeed = document.getElementById('camera-feed');

    if (!cameraFeed || !cameraFeed.src || cameraFeed.src === '') {
        alert('No camera feed available');
        return;
    }

    try {
        // Create canvas to convert image
        const canvas = document.createElement('canvas');
        canvas.width = cameraFeed.naturalWidth || 640;
        canvas.height = cameraFeed.naturalHeight || 480;

        const ctx = canvas.getContext('2d');
        ctx.drawImage(cameraFeed, 0, 0);

        // Download image
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
        const filename = `robot_snapshot_${timestamp}.png`;

        canvas.toBlob(blob => {
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = filename;
            a.click();
            URL.revokeObjectURL(url);

            window.addLog && window.addLog('success', `Snapshot saved: ${filename}`);
        }, 'image/png');

    } catch (error) {
        console.error('Error taking snapshot:', error);
        window.addLog && window.addLog('error', 'Failed to take snapshot');
    }
}

/**
 * Request Camera Frame (fallback polling)
 */
async function requestCameraFrame() {
    try {
        const response = await fetch(`${window.CONFIG.apiBaseUrl}/api/camera/latest`);
        const data = await response.json();

        if (data.status === 'success' && data.data) {
            updateCameraFeed(data.data.frame, data.data.timestamp);
        }
    } catch (error) {
        console.error('Error requesting camera frame:', error);
    }
}

// Initialize camera when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    initCamera();
});

// Export functions to global scope
window.updateCameraFeed = updateCameraFeed;
window.takeSnapshot = takeSnapshot;