#!/usr/bin/env python3
"""
ArmMotion: Refactored motion class for ArmPi pick-and-place operations.

Extracts and abstracts the motion pipeline from ColorTracking.py,
ColorSorting.py, and ColorPalletizing.py into reusable methods.

Motion pipeline flow (basic pick-and-place):
    1. init_position()      - Move arm to home/ready position
    2. move_above_block()   - Position arm above the detected block
    3. open_gripper()       - Open the gripper
    4. rotate_gripper()     - Rotate gripper to match block angle
    5. lower_to_block()     - Lower arm to grasp height
    6. close_gripper()      - Close gripper to grab block
    7. raise_arm()          - Lift block up
    8. move_to_place()      - Move arm to the placement coordinate
    9. lower_to_place()     - Lower arm to placement height
    10. open_gripper()      - Release block
    11. raise_arm()         - Lift arm clear
    12. init_position()     - Return to home

Additional functionality from ColorSorting/ColorPalletizing:
    - Color-based sorting to different coordinates
    - Height-stacking (palletizing) with automatic z-level tracking

High-level convenience methods:
    - pick_block()          - Steps 2-7 combined
    - place_block()         - Steps 8-12 combined
    - pick_and_place()      - Full pick-place cycle
    - pick_and_sort()       - Pick and place at color-specific location
    - pick_and_stack()      - Pick and stack with height tracking
"""
import sys
sys.path.append('/home/pi/ArmPi/')

import time
from ArmIK.Transform import getAngle
from ArmIK.ArmMoveIK import ArmIK
import HiwonderSDK.Board as Board


class ArmMotion:
    """Controls ArmPi arm for pick-and-place operations.

    Wraps servo commands and inverse kinematics into high-level
    motion primitives.
    """

    # Default placement coordinates for color sorting (x, y, z) in cm
    SORT_COORDINATES = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5,  6 - 0.5, 1.5),
        'blue':  (-15 + 0.5,  0 - 0.5, 1.5),
    }

    # Default stacking coordinate (all colors go to same x,y)
    STACK_COORDINATE = (-15 + 1, -7 - 0.5, 1.5)

    # Height increment per stacked block
    STACK_HEIGHT_INCREMENT = 2.5

    # Max blocks per stack before resetting
    MAX_STACK_HEIGHT = 3

    def __init__(self, gripper_close_pulse=500):
        """
        Args:
            gripper_close_pulse: Servo1 pulse width when gripper is closed.
                                 May need adjustment per robot.
        """
        self.ak = ArmIK()
        self.gripper_close = gripper_close_pulse
        self._is_running = True

        # Stacking state (for palletizing)
        self._stack_counts = {'red': 0, 'green': 0, 'blue': 0}

    @property
    def is_running(self):
        return self._is_running

    @is_running.setter
    def is_running(self, value):
        self._is_running = value

    def _check_running(self):
        """Check if we should continue. Returns False if stopped."""
        return self._is_running

    # ------------------------------------------------------------------ #
    # Low-level primitives
    # ------------------------------------------------------------------ #

    def set_buzzer(self, duration=0.1):
        """Beep the buzzer for the given duration (seconds)."""
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(duration)
        Board.setBuzzer(0)

    def set_rgb(self, color):
        """Set the expansion board RGB LEDs to match a color name."""
        rgb_map = {
            'red':   (255, 0, 0),
            'green': (0, 255, 0),
            'blue':  (0, 0, 255),
        }
        r, g, b = rgb_map.get(color, (0, 0, 0))
        Board.RGB.setPixelColor(0, Board.PixelColor(r, g, b))
        Board.RGB.setPixelColor(1, Board.PixelColor(r, g, b))
        Board.RGB.show()

    def init_position(self):
        """Move arm to the home/ready position."""
        Board.setBusServoPulse(1, self.gripper_close - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.ak.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def open_gripper(self, width=280, duration=500):
        """Open the gripper.

        Args:
            width: How far to open (pulse offset from close position).
            duration: Servo move time in ms.
        """
        Board.setBusServoPulse(1, self.gripper_close - width, duration)

    def close_gripper(self, duration=500):
        """Close the gripper to grasp an object."""
        Board.setBusServoPulse(1, self.gripper_close, duration)

    def rotate_gripper(self, angle_pulse, duration=500):
        """Rotate the gripper (servo 2) to the given pulse angle.

        Args:
            angle_pulse: Servo2 pulse width (use getAngle() to compute).
            duration: Servo move time in ms.
        """
        Board.setBusServoPulse(2, angle_pulse, duration)

    def reset_gripper_rotation(self, duration=500):
        """Reset gripper rotation to center (500)."""
        Board.setBusServoPulse(2, 500, duration)

    def move_to(self, x, y, z, move_time=None):
        """Move arm end-effector to world coordinates (x, y, z) in cm.

        Args:
            x, y, z: Target coordinates in cm.
            move_time: Movement duration in ms. None for auto-calculated.

        Returns:
            (servos, alpha, movetime) on success, or False if unreachable.
        """
        result = self.ak.setPitchRangeMoving((x, y, z), -90, -90, 0, move_time)
        return result

    def move_and_wait(self, x, y, z, move_time=None):
        """Move to coordinates and wait for the movement to complete.

        Returns:
            True if reached, False if unreachable.
        """
        result = self.move_to(x, y, z, move_time)
        if result is False:
            return False
        time.sleep(result[2] / 1000)
        return True

    # ------------------------------------------------------------------ #
    # Mid-level operations
    # ------------------------------------------------------------------ #

    def move_above_block(self, world_x, world_y, height=7):
        """Move the arm above a block at the given world coordinates.

        Args:
            world_x, world_y: Block position in cm.
            height: Height above the table in cm.

        Returns:
            True if reachable, False otherwise.
        """
        return self.move_and_wait(world_x, world_y, height)

    def lower_to_block(self, world_x, world_y, grasp_height=2):
        """Lower the arm to grasp height above the block.

        Args:
            world_x, world_y: Block position in cm.
            grasp_height: Height to lower to in cm.
        """
        self.move_to(world_x, world_y, grasp_height, 1000)
        time.sleep(1.5)

    def raise_arm(self, world_x, world_y, height=12, move_time=1000):
        """Raise the arm above the current position.

        Args:
            world_x, world_y: Current position in cm.
            height: Height to raise to in cm.
            move_time: Duration in ms.
        """
        self.reset_gripper_rotation()
        self.move_to(world_x, world_y, height, move_time)
        time.sleep(1)

    # ------------------------------------------------------------------ #
    # High-level: Pick up a block
    # ------------------------------------------------------------------ #

    def pick_block(self, world_x, world_y, rotation_angle, grasp_height=2):
        """Pick up a block at the given coordinates.

        Sequence: move above -> open gripper -> rotate -> lower -> close -> raise.

        Args:
            world_x, world_y: Block world coordinates in cm.
            rotation_angle: Block rotation angle (from minAreaRect).
            grasp_height: Height at which to grasp (cm).

        Returns:
            True if successful, False if position unreachable or stopped.
        """
        # Move above the block
        if not self._check_running():
            return False
        reachable = self.move_above_block(world_x, world_y)
        if not reachable:
            return False

        # Open gripper and rotate to match block angle
        if not self._check_running():
            return False
        gripper_angle = getAngle(world_x, world_y, rotation_angle)
        self.open_gripper()
        self.rotate_gripper(gripper_angle)
        time.sleep(0.8)

        # Lower to grasp height
        if not self._check_running():
            return False
        self.lower_to_block(world_x, world_y, grasp_height)

        # Close gripper to grab block
        if not self._check_running():
            return False
        self.close_gripper()
        time.sleep(1)

        # Raise arm with block
        if not self._check_running():
            return False
        self.raise_arm(world_x, world_y)

        return True

    # ------------------------------------------------------------------ #
    # High-level: Place a block
    # ------------------------------------------------------------------ #

    def place_block(self, place_x, place_y, place_z):
        """Place a held block at the given coordinates.

        Sequence: move above target -> rotate gripper -> lower above -> lower to z -> open -> raise -> init.

        Args:
            place_x, place_y, place_z: Placement coordinates in cm.

        Returns:
            True if successful, False if stopped.
        """
        # Move above placement position
        if not self._check_running():
            return False
        self.move_and_wait(place_x, place_y, 12)

        # Rotate gripper for placement
        if not self._check_running():
            return False
        gripper_angle = getAngle(place_x, place_y, -90)
        self.rotate_gripper(gripper_angle)
        time.sleep(0.5)

        # Lower to just above placement height
        if not self._check_running():
            return False
        self.move_to(place_x, place_y, place_z + 3, 500)
        time.sleep(0.5)

        # Lower to exact placement height
        if not self._check_running():
            return False
        self.move_to(place_x, place_y, place_z, 1000)
        time.sleep(0.8)

        # Release block
        if not self._check_running():
            return False
        self.open_gripper(width=200)
        time.sleep(0.8)

        # Raise arm clear
        if not self._check_running():
            return False
        self.move_to(place_x, place_y, 12, 800)
        time.sleep(0.8)

        # Return to home
        self.init_position()
        time.sleep(1.5)

        return True

    # ------------------------------------------------------------------ #
    # Convenience: Full pick-and-place cycle
    # ------------------------------------------------------------------ #

    def pick_and_place(self, world_x, world_y, rotation_angle,
                       place_x, place_y, place_z, color=None):
        """Full cycle: pick block from detected position and place at target.

        Args:
            world_x, world_y: Block coordinates (from perception).
            rotation_angle: Block rotation (from perception).
            place_x, place_y, place_z: Target placement coordinates.
            color: Optional color name for LED/buzzer feedback.

        Returns:
            True if successful, False otherwise.
        """
        if color:
            self.set_rgb(color)
            self.set_buzzer(0.1)

        success = self.pick_block(world_x, world_y, rotation_angle)
        if not success:
            return False

        success = self.place_block(place_x, place_y, place_z)

        # Clear LED
        self.set_rgb('none')
        return success

    # ------------------------------------------------------------------ #
    # Color sorting (from ColorSorting.py)
    # ------------------------------------------------------------------ #

    def pick_and_sort(self, world_x, world_y, rotation_angle, color,
                      sort_coordinates=None):
        """Pick a block and place it at the color-specific sorting location.

        Args:
            world_x, world_y: Block coordinates.
            rotation_angle: Block rotation.
            color: Detected color name ('red', 'green', or 'blue').
            sort_coordinates: Optional dict overriding default sort positions.

        Returns:
            True if successful, False otherwise.
        """
        coords = sort_coordinates or self.SORT_COORDINATES
        if color not in coords:
            return False

        px, py, pz = coords[color]
        return self.pick_and_place(world_x, world_y, rotation_angle,
                                   px, py, pz, color=color)

    # ------------------------------------------------------------------ #
    # Stacking / palletizing (from ColorPalletizing.py)
    # ------------------------------------------------------------------ #

    def get_stack_height(self, color):
        """Get the current stacking z-height for a color.

        Returns:
            (z_height, needs_clear): z_height is the placement height,
            needs_clear is True if the stack is full and the area should
            be cleared before placing.
        """
        count = self._stack_counts.get(color, 0)
        base_z = self.STACK_COORDINATE[2]
        z = base_z + count * self.STACK_HEIGHT_INCREMENT
        needs_clear = (count >= self.MAX_STACK_HEIGHT)

        return z, needs_clear

    def pick_and_stack(self, world_x, world_y, rotation_angle, color,
                       stack_coordinate=None, clear_callback=None):
        """Pick a block and stack it at the stacking location.

        Automatically tracks stacking height per color and resets
        when the stack reaches MAX_STACK_HEIGHT.

        Args:
            world_x, world_y: Block coordinates.
            rotation_angle: Block rotation.
            color: Detected color name.
            stack_coordinate: Optional (x, y, z) base coordinate override.
            clear_callback: Optional callable invoked when stack is full
                           (gives user time to clear the area). If None,
                           sleeps for 3 seconds.

        Returns:
            True if successful, False otherwise.
        """
        coord = stack_coordinate or self.STACK_COORDINATE
        z, needs_clear = self.get_stack_height(color)

        if needs_clear:
            # Stack is full — reset height and wait for area to be cleared
            self._stack_counts[color] = 0
            z = coord[2]
            if clear_callback:
                clear_callback(color)
            else:
                time.sleep(3)

        self.set_rgb(color)
        self.set_buzzer(0.1)

        success = self.pick_block(world_x, world_y, rotation_angle)
        if not success:
            return False

        success = self.place_block(coord[0], coord[1], z)

        if success:
            self._stack_counts[color] = self._stack_counts.get(color, 0) + 1

        self.set_rgb('none')
        return success

    def reset_stack_counts(self):
        """Reset all stacking height counters."""
        self._stack_counts = {'red': 0, 'green': 0, 'blue': 0}

    # ------------------------------------------------------------------ #
    # Shutdown
    # ------------------------------------------------------------------ #

    def stop(self):
        """Stop the arm and return to a safe position."""
        self._is_running = False
        self.open_gripper(width=70, duration=300)
        time.sleep(0.5)
        self.reset_gripper_rotation()
        self.ak.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(1.5)
