import logging

from helper.logging_config import setup_logging
setup_logging()

logger = logging.getLogger(__spec__.name if __spec__ else __name__)

import atexit
from picarx import Picarx
from picarx.core.edge_detector import Edge_Detector
from picarx.sensing.grayscale_sensing import Grayscale_Sensing
from collections import deque

class Edge_Detector_Controller(object):

    DEFAULT_SPEED = 10
    MAX_STEERING_ANGLE = 30.0

    steering_angles_history: deque[float] = deque(maxlen=30)
    MAX_STEERING_ANGLE_DIFF = 2.0
    
    def __init__(self, scaling_factor: float = 2.0, threshold: int = 600, polarity: int = 0):
        self.edge_detector = Edge_Detector(threshold, polarity)
        self.gs_sensing = Grayscale_Sensing()
        self.scaling_factor = scaling_factor
        logger.info("Edge Detector Controller initialized")

    def run(self, px: Picarx, steering_direction: float = 0.0):
        try:
            steering_angle = max(-self.MAX_STEERING_ANGLE, min(self.MAX_STEERING_ANGLE, steering_direction * self.scaling_factor * self.MAX_STEERING_ANGLE))

            if self.steering_angles_history:
                last_angle = self.steering_angles_history[-1]
                angle_diff = steering_angle - last_angle
                if abs(angle_diff) > self.MAX_STEERING_ANGLE_DIFF:
                    steering_angle = last_angle + (self.MAX_STEERING_ANGLE_DIFF if angle_diff > 0 else -self.MAX_STEERING_ANGLE_DIFF)

            self.steering_angles_history.append(steering_angle)
            
            logger.info(f"Relative Steering Direction: {steering_direction}, Steering angle: {steering_angle}")
            
            px.set_dir_servo_angle(-steering_angle)
            px.forward(self.DEFAULT_SPEED * (1 - min(abs(steering_direction*self.scaling_factor), 0.9)))
        except KeyboardInterrupt:
            logger.info("Exiting edge detector controller.")
            px.close()

if __name__ == "__main__":
    px = Picarx()
    atexit.register(px.close)
    gs_sensing = Grayscale_Sensing()
    edge_detector = Edge_Detector(threshold=600, polarity=0)
    edge_detector_controller = Edge_Detector_Controller()
    edge = edge_detector.detect_edges(gs_sensing.read_values())
    edge_detector_controller.run(px, 0.1)
    print(edge_detector_controller.steering_angles_history)


