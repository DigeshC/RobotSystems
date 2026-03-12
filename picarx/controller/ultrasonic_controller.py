import logging

from helper.logging_config import setup_logging
setup_logging()

logger = logging.getLogger(__spec__.name if __spec__ else __name__)

from picarx import Picarx


class Ultrasonic_Controller(object):

    DEFAULT_SPEED = 10

    def __init__(self, speed: int = DEFAULT_SPEED):
        self.speed = speed
        logger.info("Ultrasonic controller initialized with speed=%d", speed)

    def run(self, px: Picarx, is_clear: bool):
        """Move forward if clear, stop if obstacle detected."""
        if is_clear:
            px.forward(self.speed)
            logger.debug("Path clear, moving forward at speed %d", self.speed)
        else:
            px.stop()
            logger.info("Obstacle detected, stopping")
