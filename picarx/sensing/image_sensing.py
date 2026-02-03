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
    from vilib import Vilib
    backend = "vilib"
except Exception:
    Vilib = None
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
        backend: str = "vilib",
        warmup_s: float = 0.5,
    ):
        atexit.register(self.close)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.device_index = int(device_index)
        self.warmup_s = float(max(0.0, warmup_s))

        self.backend = backend.lower().strip()
        if self.backend not in ("vilib", "opencv"):
            raise ValueError("backend must be one of: 'vilib', 'opencv'")

        self._cap: Optional[cv2.VideoCapture] = None
        self._vilib_inited = False

        logger.info("Image sensing module initializing (backend=%s)", self.backend)
        self._start()

    def _start(self) -> None:
        use_backend = self.backend
        if use_backend == "vilib":
            if Vilib is None:
                logger.warning("Vilib not available; falling back to OpenCV backend")
                use_backend = "opencv"
            else:
                self._start_vilib()
                self.backend = "vilib"
                return

        # fallback
        self._start_opencv()
        self.backend = "opencv"

    def _start_vilib(self) -> None:
        """
        Start SunFounder Vilib stream.
        Vilib keeps an internal frame buffer accessible via Vilib.camera_frame.
        """
        try:
            Vilib.camera_start(vflip=False, hflip=False)
            Vilib.display(False)  # no GUI window
            self._vilib_inited = True

            for fn_name, args in [
                ("camera_resize", (self.width, self.height)),
                ("camera_set_fps", (self.fps,)),
            ]:
                fn = getattr(Vilib, fn_name, None)
                if callable(fn):
                    try:
                        fn(*args)
                    except Exception:
                        pass

            if self.warmup_s > 0:
                time.sleep(self.warmup_s)

            logger.info("Vilib camera started (%dx%d @ %dfps target)", self.width, self.height, self.fps)
        except Exception as e:
            logger.exception("Failed to start Vilib camera: %s", e)
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

        if self.backend == "vilib":
            # Vilib stores the latest frame in Vilib.camera_frame (BGR ndarray)
            try:
                frame = getattr(Vilib, "camera_frame", None)
                if frame is None:
                    logger.debug("Vilib frame not ready yet")
                    return None
            except Exception:
                logger.debug("Failed to access Vilib frame buffer")
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

        if self.backend == "vilib" and self._vilib_inited and Vilib is not None:
            # Vilib camera_stop exists in most versions
            stop_fn = getattr(Vilib, "camera_stop", None)
            if callable(stop_fn):
                try:
                    stop_fn()
                except Exception:
                    pass
            self._vilib_inited = False

        logger.info("Image sensing module closed")


if __name__ == "__main__":
    sensing = Image_Sensing(backend="vilib", width=640, height=480, fps=30, device_index=0)
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
