#!/usr/bin/env python3
"""
Demo: Color sorting using ColorPerception + ArmMotion.

Detects red, green, and blue blocks and sorts them to color-specific
locations (replicates ColorSorting.py behavior with refactored classes).
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
        target_colors=('red', 'green', 'blue'),
        stability_distance=0.5,
        stability_duration=1.0,
        color_vote_count=3,
    )
    motion = ArmMotion()
    motion.init_position()

    camera = Camera.Camera()
    camera.camera_open()

    print("Sorting demo — detecting red/green/blue blocks. Press ESC to quit.")

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
                        print(f"[SORT] {color} block at ({sx:.1f}, {sy:.1f})")

                        picking = True
                        success = motion.pick_and_sort(sx, sy, angle, color)

                        if success:
                            print(f"[DONE] Sorted {color} block")
                        else:
                            print(f"[FAIL] Could not sort {color} block")

                        perception.reset()
                        picking = False
                else:
                    display = frame
            else:
                display = frame

            cv2.imshow('Color Sorting Demo', display)
            if cv2.waitKey(1) == 27:
                break
    finally:
        motion.stop()
        camera.camera_close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
