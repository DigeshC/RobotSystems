#!/usr/bin/env python3
"""
Demo: Basic pick-and-place using ColorPerception + ArmMotion.

Detects a red block, picks it up, and places it at the red sorting location.
Combines the refactored perception and motion classes.
Press ESC to exit.
"""
import sys
sys.path.append('/home/pi/ArmPi/')

import cv2
import Camera
from color_perception import ColorPerception
from arm_motion import ArmMotion


def main():
    perception = ColorPerception(
        target_colors=('red',),
        stability_distance=0.3,
        stability_duration=1.5,
    )
    motion = ArmMotion()
    motion.init_position()

    camera = Camera.Camera()
    camera.camera_open()

    print("Pick-and-place demo — detecting red blocks. Press ESC to quit.")

    picking = False

    try:
        while True:
            img = camera.frame
            if img is None:
                continue

            frame = img.copy()

            if not picking:
                result = perception.process_frame(frame)

                if result is not None:
                    display = result['annotated_img']

                    if result['stable_position'] is not None:
                        sx, sy = result['stable_position']
                        angle = result['rotation_angle']
                        color = result['color']
                        print(f"[PICK] {color} block stable at ({sx:.1f}, {sy:.1f}), angle={angle:.1f}")

                        picking = True
                        success = motion.pick_and_sort(sx, sy, angle, color)

                        if success:
                            print(f"[DONE] Placed {color} block successfully")
                        else:
                            print(f"[FAIL] Could not complete pick-and-place")

                        perception.reset()
                        picking = False
                else:
                    display = frame
            else:
                display = frame

            cv2.imshow('Pick and Place Demo', display)
            if cv2.waitKey(1) == 27:
                break
    finally:
        motion.stop()
        camera.camera_close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
