import logging

from helper.logging_config import setup_logging
setup_logging()

logger = logging.getLogger(__spec__.name if __spec__ else __name__)

from picarx import Picarx

import atexit

on_the_robot = None
try:
    from robot_hat import ADC
    on_the_robot = True
except ImportError:
    # import sys
    # sys.path.append(os.path.abspath(os.path.join(
    #     os.path.dirname(__file__), '..')))
    from sim_robot_hat import ADC
    on_the_robot = False

from picarx.sensing.sensing import Sensing

class Grayscale_Sensing(Sensing):

    REFERENCE_DEFAULT = [1000]*3

    def __init__(self, pin0: str = 'A0', pin1: str = 'A1', pin2: str = 'A2', reference: int = None):
        self.pins = (ADC(pin0), ADC(pin1), ADC(pin2))
        for i, pin in enumerate(self.pins):
            if not isinstance(pin, ADC):
                raise TypeError(f"pin{i} must be robot_hat.ADC")
        self._reference = self.REFERENCE_DEFAULT
        logger.info("Sensing module initialized")

    
    def read_values(self):
        values = [pin.read() for pin in self.pins]
        logger.debug(f"Grayscale sensor readings: {values}")
        return values
    

if __name__ == "__main__":
    px = Picarx()
    atexit.register(px.close)
    gs_sensing = Grayscale_Sensing()
    try:
        while True:
            gs_values = gs_sensing.read_values()
            logger.info(f"Grayscale values: {gs_values}")
    except KeyboardInterrupt:
        logger.info("Exiting grayscale sensing test.")
    