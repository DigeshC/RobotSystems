import time
import logging
from logdecorator import log_on_start, log_on_end, log_on_error

from helper.logging_config import setup_logging
setup_logging()

logger = logging.getLogger(__spec__.name if __spec__ else __name__)

from picarx import Picarx

@log_on_start(logging.DEBUG, "drive(speed={speed}, duration={duration_s}, steer={steering_deg})", logger=logger)
@log_on_end(logging.DEBUG, "drive finished", logger=logger)
@log_on_error(logging.ERROR, "drive failed", logger=logger)
def drive(px: Picarx, speed: int, duration_s: float, steering_deg: float = 0.0):
    px.set_dir_servo_angle(steering_deg)

    if speed >= 0:
        px.forward(abs(speed))
    else:
        px.backward(abs(speed))

    time.sleep(max(0.0, duration_s))
    px.stop()


@log_on_start(logging.DEBUG, "forward_straight(speed={speed}, duration={duration_s})", logger=logger)
@log_on_end(logging.DEBUG, "forward_straight finished", logger=logger)
def forward_straight(px: Picarx, speed: int = 40, duration_s: float = 1.0):
    drive(px, speed=abs(speed), duration_s=duration_s, steering_deg=0.0)


@log_on_start(logging.DEBUG, "backward_straight(speed={speed}, duration={duration_s})", logger=logger)
@log_on_end(logging.DEBUG, "backward_straight finished", logger=logger)
def backward_straight(px: Picarx, speed: int = 40, duration_s: float = 1.0):
    drive(px, speed=-abs(speed), duration_s=duration_s, steering_deg=0.0)


@log_on_start(logging.DEBUG, "forward_turn(steer={steering_deg}, speed={speed}, duration={duration_s})", logger=logger)
@log_on_end(logging.DEBUG, "forward_turn finished", logger=logger)
def forward_turn(px: Picarx, steering_deg: float, speed: int = 35, duration_s: float = 1.0):
    drive(px, speed=abs(speed), duration_s=duration_s, steering_deg=steering_deg)


@log_on_start(logging.DEBUG, "backward_turn(steer={steering_deg}, speed={speed}, duration={duration_s})", logger=logger)
@log_on_end(logging.DEBUG, "backward_turn finished", logger=logger)
def backward_turn(px: Picarx, steering_deg: float, speed: int = 35, duration_s: float = 1.0):
    drive(px, speed=-abs(speed), duration_s=duration_s, steering_deg=steering_deg)


def parallel_park_right(px: Picarx, speed: int = 35, t1=1.0, t2=1.0, t3=0.6):
    logger.info("Maneuver: parallel_park_right")

    steering_deg = px.DIR_MAX
    backward_turn(px, +steering_deg, speed, t1)
    backward_turn(px, -steering_deg, speed, t2)
    forward_straight(px, speed, t3)
    px.set_dir_servo_angle(0.0)


def parallel_park_left(px: Picarx, speed: int = 35, t1=1.0, t2=1.0, t3=0.6):
    logger.info("Maneuver: parallel_park_left")

    steering_deg = px.DIR_MAX
    backward_turn(px, -steering_deg, speed, t1)
    backward_turn(px, +steering_deg, speed, t2)
    forward_straight(px, speed, t3)
    px.set_dir_servo_angle(0.0)


def k_turn_left(px: Picarx, speed: int = 35, t1=1.0, t2=1.0, t3=1.0):
    logger.info("Maneuver: k_turn_left")

    steering_deg = px.DIR_MAX
    forward_turn(px, -steering_deg, speed, t1)
    backward_turn(px, +steering_deg, speed, t2)
    forward_turn(px, -steering_deg, speed, t3)
    px.set_dir_servo_angle(0.0)


def k_turn_right(px: Picarx, speed: int = 35, t1=1.0, t2=1.0, t3=1.0):
    logger.info("Maneuver: k_turn_right")

    steering_deg = px.DIR_MAX
    forward_turn(px, +steering_deg, speed, t1)
    backward_turn(px, -steering_deg, speed, t2)
    forward_turn(px, +steering_deg, speed, t3)
    px.set_dir_servo_angle(0.0)


if __name__ == "__main__":
    px = Picarx()
    try:
        forward_straight(px, 40, 1.0)
        forward_turn(px, 20, 35, 1.0)
        backward_straight(px, 40, 1.0)

        parallel_park_right(px)
        k_turn_left(px)

    finally:
        px.stop()
        px.close()
