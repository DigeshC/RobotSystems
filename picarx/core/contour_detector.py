import logging

from helper.logging_config import setup_logging
import numpy as np
import cv2

from picarx.core.detector import Detector
from picarx.sensing.image_sensing import Image_Sensing

import time


class Contour_Detector(Detector):

    def __init__(
        self,
        polarity: int = 0,
        threshold: int = 120,
        min_contour_area: int = 300,
        debug_draw: bool = False,
    ):
        setup_logging()
        self.logger = logging.getLogger(__spec__.name if __spec__ else __name__)
        self.logger.info("Contour Detector initialized")

        self.polarity = polarity
        self.threshold = threshold
        self.min_contour_area = min_contour_area
        self.debug_draw = debug_draw
        self.last_debug_image: np.ndarray | None = None

    '''
    Detect black or white contour line on the center half of the image.

    Returns:
        float in range [-1, 1]
        +1 => line far left
        -1 => line far right
    '''
    def detect(self, image: np.ndarray) -> float:
        if image is None:
            return 0.0

        h, w = image.shape[:2]

        # --- Center Bottom Half ROI ---
        roi = image[h // 2 : h , w // 4 : 3 * w // 4]

        # --- Grayscale ---
        if roi.ndim == 3:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi

        # --- Binary threshold ---
        thresh_type = cv2.THRESH_BINARY_INV if self.polarity == 0 else cv2.THRESH_BINARY
        _, binary = cv2.threshold(gray, self.threshold, 255, thresh_type)

        # --- Morphology cleanup ---
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # --- Find contours ---
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.logger.debug("No contours found")
            return 0.0

        # --- Largest contour ---
        contour = max(contours, key=cv2.contourArea)
        self.logger.debug("Found %d contours, largest area=%.2f", len(contours), cv2.contourArea(contour))
        if cv2.contourArea(contour) < self.min_contour_area:
            self.logger.debug("Contour too small")
            return 0.0

        # --- Centroid ---
        M = cv2.moments(contour)
        if M["m00"] == 0:
            return 0.0

        cx = int(M["m10"] / M["m00"])
        roi_center_x = roi.shape[1] / 2.0

        # Normalize to [-1, 1]
        steering = -(cx - roi_center_x) / roi_center_x
        steering = max(-1.0, min(1.0, steering))

        self.logger.debug("cx=%d steering=%.3f", cx, steering)

        # --- Debug overlay ---
        if self.debug_draw:
            debug_img = image.copy()

            x0, y0 = w // 4, h // 2
            x1, y1 = 3 * w // 4, h

            cv2.rectangle(debug_img, (x0, y0), (x1, y1), (0, 255, 255), 2)

            # Compute centroid in ROI coords
            cy = int(M["m01"] / M["m00"])

            contour_shifted = contour.copy()
            contour_shifted[:, 0, 0] += x0
            contour_shifted[:, 0, 1] += y0
            cv2.drawContours(debug_img, [contour_shifted], -1, (0, 255, 0), 2)

            cv2.circle(debug_img, (int(cx + x0), int(cy + y0)), 6, (0, 0, 255), -1)

            self.last_debug_image = debug_img

        return steering

if __name__ == "__main__":
    setup_logging()
    logger = logging.getLogger(__spec__.name if __spec__ else __name__)

    camera = Image_Sensing(
        backend="vilib",
        width=640,
        height=480,
        fps=30,
        device_index=0,
        warmup_s=0.5,
    )

    detector = Contour_Detector(
        polarity=0,
        threshold=120,
        min_contour_area=300,
        debug_draw=True,
    )

    target_hz = 10.0
    dt = 1.0 / target_hz

    logger.info("Starting contour detection test. Press 'q' to quit.")

    try:
        while True:
            frame = camera.read_values()
            if frame is None:
                time.sleep(0.01)
                continue

            steering = detector.detect(frame)
            logger.info("steering=%.3f", steering)

            # Show debug frame if enabled, else show raw
            to_show = detector.last_debug_image if detector.last_debug_image is not None else frame
            cv2.imshow("contour_debug", to_show)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            time.sleep(dt)

    except KeyboardInterrupt:
        logger.info("Exiting contour detection test.")
    finally:
        camera.close()
        cv2.destroyAllWindows()