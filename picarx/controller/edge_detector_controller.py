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

    MAX_SPEED = 10
    MAX_STEERING_ANGLE = 30.0
    
    def __init__(self, scaling_factor: float = 2.0, history_len: int = 30, max_angle_diff: float = 2.0, start_speed: int = MAX_SPEED * 0.25):
        self.scaling_factor = scaling_factor

        self.steering_angles_history: deque[float] = deque(maxlen=history_len)
        self.max_steering_angle_diff = max_angle_diff
        self.start_speed = start_speed
        self.last_speed = start_speed
        self.sharp_turns = False
        self.speed_step = (self.MAX_SPEED - self.start_speed) / 10.0
        logger.info("Edge Detector Controller initialized")

    def clamp(self, x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))


    def run(self, px: Picarx, steering_direction: float = 0.0):
        try:
            calculated_steering_angle = steering_direction * self.scaling_factor * self.MAX_STEERING_ANGLE
            steering_angle = self.clamp(calculated_steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)

            if self.steering_angles_history:
                last_angle = self.steering_angles_history[-1]
                angle_diff = steering_angle - last_angle
                if abs(angle_diff) > self.max_steering_angle_diff:
                    self.sharp_turns = True
                    steering_angle = last_angle + (self.max_steering_angle_diff if angle_diff > 0 else -self.max_steering_angle_diff)

            self.steering_angles_history.append(steering_angle)
                        
            px.set_dir_servo_angle(-steering_angle)
            speed_factor = 1 - min(abs(steering_direction * self.scaling_factor), 0.9)
            # Slow down on sharp turns
            # Increase speed slowly if not sharp turns
            if self.sharp_turns:
                px.forward(self.start_speed * speed_factor)
            else:
                self.next_speed = max(self.start_speed, self.last_speed + self.speed_step)
                speed = self.clamp(self.next_speed, self.start_speed, self.MAX_SPEED)
                self.last_speed = speed
                px.forward(speed * speed_factor)

            logger.info(f"Relative Steering Direction: {steering_direction}, Steering angle: {steering_angle}, Speed: {self.last_speed * speed_factor}")

        except KeyboardInterrupt:
            logger.info("Exiting edge detector controller.")
            px.close()

if __name__ == "__main__":
    px = Picarx()
    atexit.register(px.close)
    gs_sensing = Grayscale_Sensing()
    edge_detector = Edge_Detector(threshold=600, polarity=0)
    edge_detector_controller = Edge_Detector_Controller()
    while True:
        edge = edge_detector.detect_edges(gs_sensing.read_values())
        edge_detector_controller.run(px, steering_direction=edge)


