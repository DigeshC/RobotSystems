import logging

from helper.logging_config import setup_logging
from picarx.core.detector import Detector

from picarx import Picarx

from picarx.sensing.grayscale_sensing import Grayscale_Sensing

import atexit

class Edge_Detector(Detector):
    
    def __init__(self, threshold: int = 600, polarity: int = 0):
        setup_logging()
        self.logger = logging.getLogger(__spec__.name if __spec__ else __name__)
        self.threshold = threshold
        self.polarity = polarity
        self.logger.info("Edge Detector initialized with threshold %d and polarity %d", threshold, polarity)

    def detect(self, sensor_values: list[int]) -> float:
        if len(sensor_values) != 3:
            raise ValueError("sensor_values must be a list of three integers")
        
        left, center, right = sensor_values
        value = 0.0
        
        if left < self.threshold and center >= self.threshold:
            value = (center-left)/center
        elif right < self.threshold and center >= self.threshold:
            value = (right-center)/center
        elif left > self.threshold and right > self.threshold and center < self.threshold:
            value = (right-left)/((right+left)/2)

        if self.polarity == 1:
            value = -value

        self.logger.debug(f"Sensor values: {sensor_values}, Detected edges: {value}")
        return value
    
if __name__ == "__main__":
    edge_detector = Edge_Detector()
    px = Picarx()
    atexit.register(px.close)
    gs_sensing = Grayscale_Sensing()
    try:
        while True:
            gs_values = gs_sensing.read_values()
            edge_value = edge_detector.detect(gs_values)
            edge_detector.logger.info(f"Grayscale values: {gs_values}, Edge value: {edge_value}")
    except KeyboardInterrupt:
        edge_detector.logger.info("Exiting edge detection test.")