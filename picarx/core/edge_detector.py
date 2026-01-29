import logging

from helper.logging_config import setup_logging
setup_logging()

logger = logging.getLogger(__spec__.name if __spec__ else __name__)

from picarx import Picarx

from picarx.sensing.sensing import Grayscale_Sensing

import atexit

class Edge_Detector(object):
    
    def __init__(self, threshold: int = 600, polarity: int = 0):
        self.threshold = threshold
        self.polarity = polarity
        logger.info("Edge Detector initialized with threshold %d and polarity %d", threshold, polarity)

    def detect_edges(self, sensor_values: list[int]) -> float:
        if len(sensor_values) != 3:
            raise ValueError("sensor_values must be a list of three integers")
        
        left, center, right = sensor_values
        value = 0.0
        
        if left < self.threshold and center >= self.threshold:
            value = (center-left)/self.center
        elif right < self.threshold and center >= self.threshold:
            value = (right-center)/self.center
        elif left > self.threshold and right > self.threshold and center < self.threshold:
            value = (right-left)/((right+left)/2)

        logger.debug(f"Sensor values: {sensor_values}, Detected edges: {value}")
        return value
    
if __name__ == "__main__":
    edge_detector = Edge_Detector()
    px = Picarx()
    atexit.register(px.close)
    gs_sensing = Grayscale_Sensing()
    try:
        while True:
            gs_values = gs_sensing.read_values()
            edge_value = edge_detector.detect_edges(gs_values)
            logger.info(f"Grayscale values: {gs_values}, Edge value: {edge_value}")
    except KeyboardInterrupt:
        logger.info("Exiting edge detection test.")