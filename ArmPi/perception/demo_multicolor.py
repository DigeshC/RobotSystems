#!/usr/bin/env python3
"""
Demo: Multi-color detection with voting (ColorSorting/Palletizing functionality).

Detects red, green, and blue blocks simultaneously, uses color voting
for robust classification, and labels them on the video display.
Press ESC to exit.
"""
import sys
sys.path.append('/home/pi/ArmPi/')

import cv2
import Camera
from color_perception import ColorPerception


def main():
    perception = ColorPerception(
        target_colors=('red', 'green', 'blue'),
        stability_distance=0.5,
        stability_duration=1.0,
        color_vote_count=3,
    )

    camera = Camera.Camera()
    camera.camera_open()

    print("Demo started — detecting red/green/blue blocks with voting. Press ESC to quit.")

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
                h, w = frame.shape[:2]
                cv2.line(frame, (0, h // 2), (w, h // 2), (0, 0, 200), 1)
                cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 0, 200), 1)
                display = frame

            cv2.imshow('Multi-Color Detection Demo', display)
            if cv2.waitKey(1) == 27:
                break
    finally:
        camera.camera_close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
