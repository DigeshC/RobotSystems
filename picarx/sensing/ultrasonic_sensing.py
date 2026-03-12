import logging

from helper.logging_config import setup_logging
setup_logging()

logger = logging.getLogger(__spec__.name if __spec__ else __name__)

from picarx import Picarx
from picarx.sensing.sensing import Sensing


class Ultrasonic_Sensing(Sensing):

    def __init__(self, px: Picarx):
        self.px = px
        logger.info("Ultrasonic sensing module initialized")

    def read_values(self):
        distance = self.px.get_distance()
        logger.debug(f"Ultrasonic sensor reading: {distance} cm")
        return distance
