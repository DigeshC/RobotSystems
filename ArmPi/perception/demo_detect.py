#!/usr/bin/env python3
"""
Demo: Use ColorPerception to detect colored blocks and label them on video.

Single-color tracking mode (like ColorTracking.py).
Press ESC to exit.
"""
import sys
sys.path.append('/home/pi/ArmPi/')

import cv2
import Camera
from color_perception import ColorPerception


def main():
    perception = ColorPerception(
        target_colors=('red',),
        stability_distance=0.3,
        stability_duration=1.5,
    )

    camera = Camera.Camera()
    camera.camera_open()

    print("Demo started — detecting 'red' blocks. Press ESC to quit.")

    try:
        while True:
            img = camera.frame
            if img is None:
                continue

            frame = img.copy()
            result = perception.process_frame(frame)

            if result is not None:
                display = result['annotated_img']
                if result['stable_position'] is not None:
                    sx, sy = result['stable_position']
                    print(f"[STABLE] {result['color']} block at "
                          f"({sx:.1f}, {sy:.1f}), "
                          f"angle={result['rotation_angle']:.1f}")
            else:
                # No detection — just draw crosshairs
                h, w = frame.shape[:2]
                cv2.line(frame, (0, h // 2), (w, h // 2), (0, 0, 200), 1)
                cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 0, 200), 1)
                display = frame

            cv2.imshow('Color Detection Demo', display)
            if cv2.waitKey(1) == 27:
                break
    finally:
        camera.camera_close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
