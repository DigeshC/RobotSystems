#!/usr/bin/env python3
"""
Demo: Block stacking (palletizing) using ColorPerception + ArmMotion.

Detects blocks and stacks them at a single location, incrementing height
automatically (replicates ColorPalletizing.py behavior with refactored classes).
Press ESC to exit.
"""
import sys
sys.path.append('/home/pi/ArmPi/')

import cv2
import Camera
from color_perception import ColorPerception
from arm_motion import ArmMotion


def on_stack_full(color):
    """Called when the stack is full and needs to be cleared."""
    print(f"[WARN] Stack full for {color} — clear the stacking area! Waiting 3s...")
    # In the original code, a text overlay is shown on the video frame.
    # Here we just wait.
    import time
    time.sleep(3)


def main():
    perception = ColorPerception(
        target_colors=('red', 'green', 'blue'),
        stability_distance=0.5,
        stability_duration=0.5,
        color_vote_count=3,
    )
    motion = ArmMotion()
    motion.init_position()

    camera = Camera.Camera()
    camera.camera_open()

    print("Stacking demo — blocks will be stacked at one location. Press ESC to quit.")

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

                        z, needs_clear = motion.get_stack_height(color)
                        print(f"[STACK] {color} block at ({sx:.1f}, {sy:.1f}), "
                              f"stack z={z:.1f}")

                        picking = True
                        success = motion.pick_and_stack(
                            sx, sy, angle, color,
                            clear_callback=on_stack_full,
                        )

                        if success:
                            print(f"[DONE] Stacked {color} block")
                        else:
                            print(f"[FAIL] Could not stack {color} block")

                        perception.reset()
                        picking = False
                else:
                    display = frame
            else:
                display = frame

            cv2.imshow('Stacking Demo', display)
            if cv2.waitKey(1) == 27:
                break
    finally:
        motion.stop()
        camera.camera_close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
