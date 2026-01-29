#!/usr/bin/env python3
import logging

from helper.logging_config import setup_logging
setup_logging()

logger = logging.getLogger(__spec__.name if __spec__ else __name__)

from picarx import Picarx
import atexit

from picarx.maneuvers import (
    forward_straight,
    backward_straight,
    forward_turn,
    backward_turn,
    parallel_park_right,
    parallel_park_left,
    k_turn_left,
    k_turn_right,
)

HELP_TEXT = """
Teleop commands:
  w  -> forward straight
  s  -> backward straight
  a  -> forward turn left   (uses px.DIR_MAX)
  d  -> forward turn right  (uses px.DIR_MAX)
  q  -> backward turn left  (uses px.DIR_MAX)
  e  -> backward turn right (uses px.DIR_MAX)

  pr -> parallel park right
  pl -> parallel park left
  kl -> K-turn left
  kr -> K-turn right

  x  -> stop
  h  -> help
  quit / exit -> quit
"""

def main():
    px = Picarx()
    atexit.register(px.close)
    logger.info("Teleop started.")
    print(HELP_TEXT)

    # Default params
    speed = 35
    duration = 1.0

    try:
        while True:
            cmd = input("teleop> ").strip().lower()

            if cmd in ("quit", "exit"):
                print("Exiting teleop.")
                break

            if cmd in ("h", "help", "?"):
                print(HELP_TEXT)
                continue

            if cmd == "":
                continue

            if cmd == "x":
                px.stop()
                print("Stopped.")
                continue

            # Map commands to maneuvers
            try:
                if cmd == "w":
                    forward_straight(px, speed=speed, duration_s=duration)

                elif cmd == "s":
                    backward_straight(px, speed=speed, duration_s=duration)

                elif cmd == "a":
                    forward_turn(px, steering_deg=-px.DIR_MAX, speed=speed, duration_s=duration)

                elif cmd == "d":
                    forward_turn(px, steering_deg=+px.DIR_MAX, speed=speed, duration_s=duration)

                elif cmd == "q":
                    backward_turn(px, steering_deg=-px.DIR_MAX, speed=speed, duration_s=duration)

                elif cmd == "e":
                    backward_turn(px, steering_deg=+px.DIR_MAX, speed=speed, duration_s=duration)

                elif cmd == "pr":
                    parallel_park_right(px, speed=speed)

                elif cmd == "pl":
                    parallel_park_left(px, speed=speed)

                elif cmd == "kl":
                    k_turn_left(px, speed=speed)

                elif cmd == "kr":
                    k_turn_right(px, speed=speed)

                else:
                    print(f"Unknown command: '{cmd}'. Type 'h' for help.")

            except Exception as ex:
                logger.exception("Maneuver failed for cmd='%s': %s", cmd, ex)
                px.stop()

    except KeyboardInterrupt:
        print("\nCtrl+C received. Exiting teleop.")

if __name__ == "__main__":
    main()
