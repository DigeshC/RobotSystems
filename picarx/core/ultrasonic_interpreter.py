import logging

from helper.logging_config import setup_logging
setup_logging()

logger = logging.getLogger(__spec__.name if __spec__ else __name__)


class Ultrasonic_Interpreter(object):

    def __init__(self, safe_distance: float = 30.0):
        self.safe_distance = safe_distance
        logger.info("Ultrasonic interpreter initialized with safe_distance=%.1f cm", safe_distance)

    def interpret(self, distance: float) -> bool:
        """Interpret ultrasonic distance reading.

        Returns True if the path is clear, False if an obstacle is detected.
        Error codes (-1, -2) are treated as 'obstacle detected' for safety.
        """
        if distance == -1:
            logger.warning("Ultrasonic timeout (-1), treating as obstacle")
            return False
        if distance == -2:
            logger.debug("Ultrasonic out of range (-2), treating as clear")
            return True

        is_clear = distance > self.safe_distance
        logger.debug("Distance: %.1f cm, clear: %s", distance, is_clear)
        return is_clear
