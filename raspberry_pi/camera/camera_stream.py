"""
Camera Stream Module
Captures and streams video from Pi Camera
"""

import logging
import time
try:
    from picamera import PiCamera
    from picamera.array import PiRGBArray
    import cv2
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False
    logging.warning("Pi Camera libraries not available")

logger = logging.getLogger('camera_stream')


class CameraStream:
    """Pi Camera video streaming"""
    
    def __init__(self, config):
        """
        Initialize camera
        
        Args:
            config: Camera configuration dictionary
        """
        self.config = config
        self.camera = None
        self.raw_capture = None
        self.enabled = CAMERA_AVAILABLE
        
        if not self.enabled:
            logger.warning("Camera not available")
            return
        
        try:
            self.camera = PiCamera()
            self.camera.resolution = tuple(config['resolution'])
            self.camera.framerate = config['fps']
            self.camera.rotation = config.get('rotation', 0)
            
            self.raw_capture = PiRGBArray(self.camera, size=tuple(config['resolution']))
            
            # Allow camera to warm up
            time.sleep(2)
            
            logger.info(f"Camera initialized: {config['resolution']}@{config['fps']}fps")
            
        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}")
            self.enabled = False
    
    def capture_frame(self):
        """
        Capture a single frame
        
        Returns:
            numpy array: Frame image or None
        """
        if not self.enabled or not self.camera:
            return None
        
        try:
            self.raw_capture.truncate(0)
            self.camera.capture(self.raw_capture, format='bgr', use_video_port=True)
            frame = self.raw_capture.array
            return frame
            
        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return None
    
    def stop(self):
        """Stop camera and cleanup"""
        if self.camera:
            self.camera.close()
            logger.info("Camera stopped")
