/**
 * Camera module: force MJPEG stream via same-origin '/camera/mjpeg'
 * No polling fallback.
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

  // Same-origin MJPEG relay provided by dashboard
  cameraFeed.src = '/camera/mjpeg';
  cameraFeed.classList.add('active');

  if (noCamera) noCamera.style.display = 'none';
  if (status) {
    status.textContent = 'Live (MJPEG)';
    status.style.background = 'rgba(16, 185, 129, 0.3)';
  }
}

document.addEventListener('DOMContentLoaded', () => {
  console.log('camera.js loaded; initializing MJPEG');
  initCamera();
});