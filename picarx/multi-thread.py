from time import sleep
from concurrent.futures import ThreadPoolExecutor
from threading import Event
from picarx import Picarx
from picarx.bus.bus import Bus
from picarx.sensing.sensing import Sensing
from picarx.sensing.image_sensing import Image_Sensing
from picarx.core.detector import Detector
from picarx.core.contour_detector import Contour_Detector
from picarx.controller.steering_controller import Steering_Controller
from picarx.sensing.sensing import Sensing

# Define shutdown event
shutdown_event = Event()
# Exception handle function
def handle_exception(future):
    exception = future.exception()
    if exception:
        print(f'Exception in worker thread: {exception}')
    # Define robot task

def sensor_task(sensor: Sensing, sensor_bus: Bus, sensor_delay: float = 0.02):
    while not shutdown_event.is_set():
        sensor_values = sensor.read_values()
        sensor_bus.write(sensor_values)
        sleep(sensor_delay)

def interpretor_task(detector: Detector, sensor_bus: Bus, interpretor_bus: Bus, interpretor_delay: float = 0.02):
    while not shutdown_event.is_set():
        sensor_values = sensor_bus.read()
        if sensor_values is not None:
            steering_direction = detector.detect(sensor_values)
            interpretor_bus.write(steering_direction)
        sleep(interpretor_delay)

def controller_task(px: Picarx, controller: Steering_Controller, interpretor_bus: Bus, controller_delay: float = 0.02):
    while not shutdown_event.is_set():
        steering_direction = interpretor_bus.read()
        if steering_direction is not None:
            controller.run(px, steering_direction)
        sleep(controller_delay)


if __name__ == '__main__':
    futures = []
    sensor = Image_Sensing()
    detector = Contour_Detector()
    controller = Steering_Controller()
    px = Picarx()
    px.set_cam_tilt_angle(-30)
    sensor_bus = Bus()
    interpretor_bus = Bus()

    with ThreadPoolExecutor(max_workers=3) as executor:
        e_sensor = executor.submit(sensor_task, sensor, sensor_bus)
        e_interpretor = executor.submit(interpretor_task, detector, sensor_bus, interpretor_bus)
        e_controller = executor.submit(controller_task, px, controller, interpretor_bus)
        e_sensor.add_done_callback(handle_exception)
        e_interpretor.add_done_callback(handle_exception)
        e_controller.add_done_callback(handle_exception)
        futures.append(e_sensor)
        futures.append(e_interpretor)
        futures.append(e_controller)
        try:
            # Keep the main thread running to response for the kill signal
            while not shutdown_event.is_set():
                sleep(1)
        except KeyboardInterrupt:
            # Trigger the shutdown event when receive the kill signal
            print('Shutting down')
            shutdown_event.set()
        finally:
            # Cancel all pending tasks
            for future in futures:
                future.cancel()
        # Ensures all threads finish
        executor.shutdown()
