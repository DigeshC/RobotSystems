#!/usr/bin/env python3
"""
Concurrent line-follower with ultrasonic obstacle avoidance using RossROS.

Control loops:
  1. Grayscale sensor -> Edge interpreter -> Steering controller (line following)
  2. Ultrasonic sensor -> Distance interpreter -> Speed controller (obstacle avoidance)
  3. Timer -> termination bus (shuts everything down after a set duration)
"""
import logging
import atexit

from helper.logging_config import setup_logging
setup_logging()

logger = logging.getLogger(__spec__.name if __spec__ else __name__)

from picarx import Picarx
from picarx.sensing.grayscale_sensing import Grayscale_Sensing
from picarx.sensing.ultrasonic_sensing import Ultrasonic_Sensing
from picarx.core.edge_detector import Edge_Detector
from picarx.core.ultrasonic_interpreter import Ultrasonic_Interpreter
from picarx.controller.edge_detector_controller import Edge_Detector_Controller
from picarx.controller.ultrasonic_controller import Ultrasonic_Controller
from picarx.rossros import (
    Bus, Producer, ConsumerProducer, Consumer, Timer, Printer, runConcurrently,
)

# --- Configuration ---
RUN_DURATION = 30       # seconds
SENSOR_DELAY = 0.05     # 50ms between sensor reads
INTERP_DELAY = 0.05     # 50ms between interpretations
CONTROL_DELAY = 0.05    # 50ms between control updates
PRINT_DELAY = 0.25      # 250ms between prints


def main():
    # --- Hardware ---
    px = Picarx()
    atexit.register(px.close)

    # --- Sensor / interpreter / controller instances ---
    gs_sensing = Grayscale_Sensing()
    edge_detector = Edge_Detector(threshold=600, polarity=0)
    edge_controller = Edge_Detector_Controller(scaling_factor=2.0, threshold=600, polarity=0)

    us_sensing = Ultrasonic_Sensing(px)
    us_interpreter = Ultrasonic_Interpreter(safe_distance=30.0)
    us_controller = Ultrasonic_Controller(speed=Edge_Detector_Controller.DEFAULT_SPEED)

    # --- Buses ---
    # Termination bus (Timer writes countdown here; all threads watch it)
    termination_bus = Bus(initial_message=False, name="Termination Bus")

    # Line-following buses
    gs_bus = Bus(initial_message=[0, 0, 0], name="Grayscale Bus")
    edge_bus = Bus(initial_message=0.0, name="Edge Bus")

    # Ultrasonic buses
    us_distance_bus = Bus(initial_message=100.0, name="Ultrasonic Distance Bus")
    us_clear_bus = Bus(initial_message=True, name="Ultrasonic Clear Bus")

    # --- Timer (Producer) ---
    timer = Timer(
        output_buses=termination_bus,
        duration=RUN_DURATION,
        delay=0.1,
        termination_buses=termination_bus,
        name="Run Timer",
    )

    # --- Line-following pipeline ---
    # Producer: read grayscale sensor values
    gs_producer = Producer(
        producer_function=gs_sensing.read_values,
        output_buses=gs_bus,
        delay=SENSOR_DELAY,
        termination_buses=termination_bus,
        name="Grayscale Sensor Producer",
    )

    # ConsumerProducer: interpret grayscale -> edge value
    edge_cp = ConsumerProducer(
        consumer_producer_function=edge_detector.detect,
        input_buses=gs_bus,
        output_buses=edge_bus,
        delay=INTERP_DELAY,
        termination_buses=termination_bus,
        name="Edge Detector Interpreter",
    )

    # Consumer: steer based on edge value
    def steer(edge_value):
        edge_controller.run(px, edge_value)
    steering_consumer = Consumer(
        consumer_function=steer,
        input_buses=edge_bus,
        delay=CONTROL_DELAY,
        termination_buses=termination_bus,
        name="Steering Controller",
    )

    # --- Ultrasonic pipeline ---
    # Producer: read ultrasonic distance
    us_producer = Producer(
        producer_function=us_sensing.read_values,
        output_buses=us_distance_bus,
        delay=SENSOR_DELAY,
        termination_buses=termination_bus,
        name="Ultrasonic Sensor Producer",
    )

    # ConsumerProducer: interpret distance -> is_clear
    us_interp_cp = ConsumerProducer(
        consumer_producer_function=us_interpreter.interpret,
        input_buses=us_distance_bus,
        output_buses=us_clear_bus,
        delay=INTERP_DELAY,
        termination_buses=termination_bus,
        name="Ultrasonic Interpreter",
    )

    # Consumer: stop/go based on is_clear
    def drive(is_clear):
        us_controller.run(px, is_clear)
    us_drive_consumer = Consumer(
        consumer_function=drive,
        input_buses=us_clear_bus,
        delay=CONTROL_DELAY,
        termination_buses=termination_bus,
        name="Ultrasonic Drive Controller",
    )

    # --- Printers (optional debug output) ---
    gs_printer = Printer(
        printer_bus=(gs_bus, edge_bus),
        delay=PRINT_DELAY,
        termination_buses=termination_bus,
        name="Grayscale Printer",
        print_prefix="GS/Edge:",
    )

    us_printer = Printer(
        printer_bus=(us_distance_bus, us_clear_bus),
        delay=PRINT_DELAY,
        termination_buses=termination_bus,
        name="Ultrasonic Printer",
        print_prefix="US Dist/Clear:",
    )

    # --- Run all concurrently ---
    logger.info("Starting concurrent control for %d seconds", RUN_DURATION)
    runConcurrently([
        timer,
        # Line-following loop
        gs_producer,
        edge_cp,
        steering_consumer,
        # Ultrasonic obstacle avoidance loop
        us_producer,
        us_interp_cp,
        us_drive_consumer,
        # Debug printers
        gs_printer,
        us_printer,
    ])

    px.stop()
    logger.info("Concurrent control finished")


if __name__ == "__main__":
    main()
