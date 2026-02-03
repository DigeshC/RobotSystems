# image_sensing.py
import logging
import time
import atexit
from typing import Optional

from helper.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__spec__.name if __spec__ else __name__)

try:
    # SunFounder PiCar-X camera helper (works on the robot when the stack is installed)
    from picamera2 import Picamera2
    backend = "picam"
except Exception:
    Picamera2 = None
    backend = "opencv"

import cv2
import numpy as np

from picarx.sensing.sensing import Sensing


class Image_Sensing(Sensing):

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        device_index: int = 0,
        backend: str = "picam",
        warmup_s: float = 0.5,
    ):
        atexit.register(self.close)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.device_index = int(device_index)
        self.warmup_s = float(max(0.0, warmup_s))

        self.backend = backend.lower().strip()
        if self.backend not in ("picam", "opencv"):
            raise ValueError("backend must be one of: 'picam', 'opencv'")

        self._cap: Optional[cv2.VideoCapture] = None
        self._picam_inited = False

        logger.info("Image sensing module initializing (backend=%s)", self.backend)
        self._start()

    def _start(self) -> None:
        use_backend = self.backend
        if use_backend == "picam":
            if Picamera2 is None:
                logger.warning("Picamera2 not available; falling back to OpenCV backend")
                use_backend = "opencv"
            else:
                self._start_picam()
                self.backend = "picam"
                return

        # fallback
        self._start_opencv()
        self.backend = "opencv"

    def _start_picam(self) -> None:
        """
        Start SunFounder PiCar-X camera.
        Picamera2 keeps an internal frame buffer accessible via Picamera2.capture_array().
        """
        try:
            self.picam2 = Picamera2()
            self.picam2.configure(self.picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
            self.picam2.start()
            self._picam_inited = True

            if self.warmup_s > 0:
                time.sleep(self.warmup_s)

            logger.info("Picamera2 camera started (%dx%d @ %dfps target)", self.width, self.height, self.fps)
        except Exception as e:
            logger.exception("Failed to start Picamera2 camera: %s", e)
            raise

    def _start_opencv(self) -> None:
        self._cap = cv2.VideoCapture(self.device_index)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open camera device index {self.device_index}")

        # Best-effort properties
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        self._cap.set(cv2.CAP_PROP_FPS, float(self.fps))

        if self.warmup_s > 0:
            time.sleep(self.warmup_s)

        logger.info(
            "OpenCV camera started (device=%d, %dx%d @ %dfps target)",
            self.device_index,
            self.width,
            self.height,
            self.fps,
        )

    def read_values(self) -> Optional[np.ndarray]:
        """
        Read and return the latest camera frame as a BGR numpy array.

        Returns:
            np.ndarray (H,W,3) BGR frame on success, else None
        """
        frame: Optional[np.ndarray] = None

        if self.backend == "picam":
            # Picamera2 stores the latest frame in Picamera2.capture_array()
            try:
                frame = self.picam2.capture_array()
                if frame is None:
                    logger.debug("Picamera2 frame not ready yet")
                    return None
            except Exception:
                logger.debug("Failed to access Picamera2 frame buffer")
                return None

        else:
            if self._cap is None:
                return None
            ok, frm = self._cap.read()
            if not ok or frm is None:
                logger.debug("OpenCV camera read failed")
                return None
            frame = frm

        logger.debug("Image frame shape: %s dtype=%s", frame.shape, frame.dtype)
        return frame

    def close(self) -> None:
        """
        Release resources.
        """
        if self.backend == "opencv" and self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

        if self.backend == "picam" and self._picam_inited and self.picam2 is not None:
            # Picamera2 camera_stop exists in most versions
            try:
                self.picam2.stop()
            except Exception:
                pass
            self._picam_inited = False

        logger.info("Image sensing module closed")


if __name__ == "__main__":
    sensing = Image_Sensing(backend="picam", width=640, height=480, fps=30, device_index=0)
    # atexit.register(sensing.close)

    last_t = time.time()
    frames = 0

    try:
        while True:
            frame = sensing.read_values()
            if frame is None:
                time.sleep(0.01)
                continue

            frames += 1
            now = time.time()
            if now - last_t >= 1.0:
                logger.info("Camera running (%s): ~%d fps", sensing.backend, frames)
                frames = 0
                last_t = now

            cv2.imshow("camera", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        logger.info("Exiting image sensing test.")
    finally:
        cv2.destroyAllWindows()
