/**
 * Camera Module - Handles live camera feed
 */

let lastFrameTime = 0;
let frameCount = 0;
let fpsInterval;

/**
 * Initialize Camera Feed
 */
function initCamera() {
    // Start FPS counter
    fpsInterval = setInterval(updateFPS, 1000);
    
    // Setup snapshot button
    document.getElementById('snapshot-btn').addEventListener('click', takeSnapshot);
    
    applyCameraConfig();

    window.addLog('info', 'Camera module initialized');
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

    if (mode === 'direct' && config.streamUrl) {
        directHint.style.display = 'block';
        directLink.href = config.streamUrl;
        directLink.textContent = config.streamUrl;

        // For direct streaming, set the camera img src directly
        const cameraFeed = document.getElementById('camera-feed');
        cameraFeed.src = config.streamUrl;
        cameraFeed.classList.add('active');
        const noCamera = document.getElementById('no-camera');
        noCamera.style.display = 'none';
        const status = document.getElementById('camera-status');
        status.textContent = 'Live';
        status.style.background = 'rgba(16, 185, 129, 0.3)';
    } else {
        if (directHint) directHint.style.display = 'none';
        if (directLink) directLink.href = '#';
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
        // Update image
        cameraFeed.src = `data:image/jpeg;base64,${frameBase64}`;
        cameraFeed.classList.add('active');
        noCamera.style.display = 'none';
        
        // Update status
        cameraStatus.textContent = 'Live';
        cameraStatus.style.background = 'rgba(16, 185, 129, 0.3)';
        
        // Update timestamp
        if (timestamp) {
            const time = new Date(timestamp);
            cameraTimestamp.textContent = time.toLocaleTimeString();
        }
        
        // Count frames for FPS
        frameCount++;
        lastFrameTime = Date.now();
        
    } catch (error) {
        console.error('Error updating camera feed:', error);
        window.addLog('error', 'Failed to update camera feed');
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
        fpsElement.textContent = '0 FPS';
        cameraStatus.textContent = 'No Feed';
        cameraStatus.style.background = 'rgba(239, 68, 68, 0.3)';
        
        // Hide feed, show placeholder
        const cameraFeed = document.getElementById('camera-feed');
        const noCamera = document.getElementById('no-camera');
        cameraFeed.classList.remove('active');
        noCamera.style.display = 'flex';
    } else {
        // Update FPS display
        fpsElement.textContent = `${frameCount} FPS`;
    }
    
    // Reset frame count
    frameCount = 0;
}

/**
 * Take Snapshot
 */
function takeSnapshot() {
    const cameraFeed = document.getElementById('camera-feed');
    
    if (!cameraFeed.src || cameraFeed.src === '') {
        alert('No camera feed available');
        return;
    }
    
    try {
        // Create canvas to convert image
        const canvas = document.createElement('canvas');
        canvas.width = cameraFeed.naturalWidth;
        canvas.height = cameraFeed.naturalHeight;
        
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
            
            window.addLog('success', `Snapshot saved: ${filename}`);
        }, 'image/png');
        
    } catch (error) {
        console.error('Error taking snapshot:', error);
        window.addLog('error', 'Failed to take snapshot');
    }
}

/**
 * Request Camera Frame
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
    
    // Poll for camera frames every 2 seconds as fallback
    // (WebSocket provides real-time updates)
    if (window.CONFIG?.camera?.streamMode !== 'direct' || window.CONFIG?.camera?.allowFallback) {
        setInterval(requestCameraFrame, 2000);
    }
});

// Export functions to global scope
window.updateCameraFeed = updateCameraFeed;
window.takeSnapshot = takeSnapshot;
