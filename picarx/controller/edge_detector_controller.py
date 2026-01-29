import logging

from helper.logging_config import setup_logging
setup_logging()

logger = logging.getLogger(__spec__.name if __spec__ else __name__)

import atexit
from picarx import Picarx
from picarx.core.edge_detector import Edge_Detector
from picarx.sensing.grayscale_sensing import Grayscale_Sensing

class Edge_Detector_Controller(object):

    DEFAULT_SPEED = 15
    MAX_STEERING_ANGLE = 30.0
    
    def __init__(self, scaling_factor: float = 1.0, threshold: int = 600, polarity: int = 0):
        self.edge_detector = Edge_Detector(threshold, polarity)
        self.gs_sensing = Grayscale_Sensing()
        self.scaling_factor = scaling_factor
        logger.info("Edge Detector Controller initialized")

    def run(self, px: Picarx):
        try:
            while True:
                gs_values = self.gs_sensing.read_values()
                edge_value = self.edge_detector.detect_edges(gs_values)
                steering_angle = max(-self.MAX_STEERING_ANGLE, min(self.MAX_STEERING_ANGLE, edge_value * self.scaling_factor * self.MAX_STEERING_ANGLE))
                
                logger.info(f"Grayscale values: {gs_values}, Edge value: {edge_value}, Steering angle: {steering_angle}")
                
                px.set_dir_servo_angle(steering_angle)
                px.forward(self.DEFAULT_SPEED)
        except KeyboardInterrupt:
            logger.info("Exiting edge detector controller.")
            px.close()

if __name__ == "__main__":
    px = Picarx()
    atexit.register(px.close)
    edge_detector_controller = Edge_Detector_Controller()
    edge_detector_controller.run(px)
