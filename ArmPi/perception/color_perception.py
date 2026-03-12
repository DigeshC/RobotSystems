#!/usr/bin/env python3
"""
ColorPerception: Refactored perception class for ArmPi color block detection.

Extracts and abstracts the perception pipeline from ColorTracking.py,
ColorSorting.py, and ColorPalletizing.py into reusable methods.

Perception pipeline flow:
    1. preprocess()         - Resize, blur, optional ROI masking
    2. segment_colors()     - BGR->LAB conversion, color thresholding, morphology
    3. find_largest_contour() - Contour detection, area filtering
    4. get_block_pose()     - Bounding box, world coordinates, rotation angle
    5. stabilize_detection() - Temporal averaging to confirm block is stationary
    6. annotate_frame()     - Draw contours, coordinates, color label on image

Additional functionality from ColorSorting/ColorPalletizing:
    - detect_dominant_color()  - Multi-color detection with voting (color_list)
"""
import sys
sys.path.append('/home/pi/ArmPi/')

import cv2
import math
import time
import numpy as np
from LABConfig import color_range
from CameraCalibration.CalibrationConfig import square_length
from ArmIK.Transform import convertCoordinate, getROI, getMaskROI, getCenter


# BGR color values for drawing
RANGE_RGB = {
    'red':   (0, 0, 255),
    'blue':  (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


class ColorPerception:
    """Detects and locates colored blocks in camera frames.

    Supports two modes:
        - Single-color tracking (like ColorTracking.py)
        - Multi-color detection with voting (like ColorSorting/Palletizing)
    """

    def __init__(self,
                 target_colors=('red',),
                 frame_size=(640, 480),
                 min_contour_area=300,
                 min_detect_area=2500,
                 stability_distance=0.5,
                 stability_duration=1.0,
                 color_vote_count=3):
        """
        Args:
            target_colors: Tuple of color names to detect (keys in LABConfig.color_range).
            frame_size: Resolution to resize frames to before processing.
            min_contour_area: Minimum contour area to consider valid (filters noise).
            min_detect_area: Minimum area to trigger block detection.
            stability_distance: Max movement (cm) between frames to count as "stationary".
            stability_duration: Seconds the block must be stationary before confirming.
            color_vote_count: Number of color samples for majority voting (0 to disable).
        """
        self.target_colors = target_colors
        self.frame_size = frame_size
        self.min_contour_area = min_contour_area
        self.min_detect_area = min_detect_area
        self.stability_distance = stability_distance
        self.stability_duration = stability_duration
        self.color_vote_count = color_vote_count

        # Internal state for stability tracking
        self._center_list = []
        self._count = 0
        self._last_x = 0.0
        self._last_y = 0.0
        self._t1 = 0.0
        self._timing_started = False

        # Internal state for color voting (ColorSorting/Palletizing feature)
        self._color_votes = []

        # Last confirmed detection result
        self._roi = None
        self._use_roi = False

    def reset(self):
        """Reset all internal tracking state."""
        self._center_list = []
        self._count = 0
        self._last_x = 0.0
        self._last_y = 0.0
        self._t1 = 0.0
        self._timing_started = False
        self._color_votes = []
        self._roi = None
        self._use_roi = False

    # ------------------------------------------------------------------ #
    # Step 1: Preprocess
    # ------------------------------------------------------------------ #
    def preprocess(self, img):
        """Resize frame, apply Gaussian blur, and optionally mask to ROI.

        Args:
            img: Raw BGR camera frame.

        Returns:
            Blurred (and possibly ROI-masked) frame at self.frame_size.
        """
        frame = cv2.resize(img, self.frame_size, interpolation=cv2.INTER_NEAREST)
        frame = cv2.GaussianBlur(frame, (11, 11), 11)

        # If we previously detected a block, restrict search to that region
        if self._use_roi and self._roi is not None:
            self._use_roi = False
            frame = getMaskROI(frame, self._roi, self.frame_size)

        return frame

    # ------------------------------------------------------------------ #
    # Step 2: Segment colors
    # ------------------------------------------------------------------ #
    def segment_colors(self, frame):
        """Convert to LAB and create binary masks for each target color.

        Args:
            frame: Preprocessed BGR frame.

        Returns:
            Dict mapping color name -> binary mask (after morphological cleanup).
        """
        frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        masks = {}
        kernel = np.ones((6, 6), np.uint8)

        for color_name in color_range:
            if color_name in self.target_colors:
                lo, hi = color_range[color_name]
                mask = cv2.inRange(frame_lab, lo, hi)
                # Morphological open (remove small noise) then close (fill gaps)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                masks[color_name] = mask

        return masks

    # ------------------------------------------------------------------ #
    # Step 3: Find largest contour
    # ------------------------------------------------------------------ #
    def find_largest_contour(self, mask):
        """Find the contour with the largest area in a binary mask.

        Args:
            mask: Binary image from color segmentation.

        Returns:
            (contour, area) tuple.  contour is None if no valid contour found.
        """
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

        best_contour = None
        best_area = 0

        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area > best_area:
                best_area = area
                if area > self.min_contour_area:
                    best_contour = c

        return best_contour, best_area

    # ------------------------------------------------------------------ #
    # Step 4: Get block pose (position + rotation)
    # ------------------------------------------------------------------ #
    def get_block_pose(self, contour):
        """Compute world coordinates and rotation angle from a contour.

        Args:
            contour: The detected contour (from find_largest_contour).

        Returns:
            dict with keys:
                'world_x', 'world_y': world coordinates in cm
                'rotation_angle': rotation of the block in degrees
                'rect': the minAreaRect object
                'box': the 4 corner points of the bounding box
                'roi': (x_min, x_max, y_min, y_max)
            or None if contour is None.
        """
        if contour is None:
            return None

        rect = cv2.minAreaRect(contour)
        box = np.int0(cv2.boxPoints(rect))

        roi = getROI(box)
        self._roi = roi
        self._use_roi = True

        img_cx, img_cy = getCenter(rect, roi, self.frame_size, square_length)
        world_x, world_y = convertCoordinate(img_cx, img_cy, self.frame_size)

        return {
            'world_x': world_x,
            'world_y': world_y,
            'rotation_angle': rect[2],
            'rect': rect,
            'box': box,
            'roi': roi,
        }

    # ------------------------------------------------------------------ #
    # Step 5: Stability check
    # ------------------------------------------------------------------ #
    def stabilize_detection(self, world_x, world_y):
        """Check if the detected block has been stationary long enough.

        Accumulates position samples. If the block hasn't moved more than
        stability_distance for stability_duration seconds, returns the
        averaged (stable) position. Otherwise returns None.

        Args:
            world_x: Current world X coordinate.
            world_y: Current world Y coordinate.

        Returns:
            (avg_x, avg_y) if stable, else None.
        """
        distance = math.sqrt((world_x - self._last_x) ** 2 +
                             (world_y - self._last_y) ** 2)
        self._last_x = world_x
        self._last_y = world_y

        if distance < self.stability_distance:
            self._center_list.extend((world_x, world_y))
            self._count += 1

            if not self._timing_started:
                self._timing_started = True
                self._t1 = time.time()

            if time.time() - self._t1 > self.stability_duration:
                # Block is confirmed stationary — compute averaged position
                avg = np.mean(
                    np.array(self._center_list).reshape(self._count, 2),
                    axis=0,
                )
                # Reset accumulators
                self._center_list = []
                self._count = 0
                self._timing_started = False
                return float(avg[0]), float(avg[1])
        else:
            # Block moved — reset
            self._t1 = time.time()
            self._timing_started = True
            self._center_list = []
            self._count = 0

        return None

    # ------------------------------------------------------------------ #
    # Step 6: Annotate frame
    # ------------------------------------------------------------------ #
    def annotate_frame(self, img, box, world_x, world_y, color_name,
                       label_text=None):
        """Draw detection results onto the display frame.

        Args:
            img: The frame to draw on (modified in place).
            box: 4-point bounding box from get_block_pose.
            world_x, world_y: World coordinates to display.
            color_name: Detected color name (for drawing color).
            label_text: Optional text label to draw at bottom of frame.

        Returns:
            The annotated image.
        """
        draw_color = RANGE_RGB.get(color_name, RANGE_RGB['black'])

        # Draw crosshairs
        h, w = img.shape[:2]
        cv2.line(img, (0, h // 2), (w, h // 2), (0, 0, 200), 1)
        cv2.line(img, (w // 2, 0), (w // 2, h), (0, 0, 200), 1)

        # Draw bounding box and coordinates
        if box is not None:
            cv2.drawContours(img, [box], -1, draw_color, 2)
            text_pos = (min(box[0, 0], box[2, 0]), box[2, 1] - 10)
            cv2.putText(img, f'({world_x},{world_y})', text_pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 1)

        # Draw color label at bottom
        if label_text is not None:
            cv2.putText(img, label_text, (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)

        return img

    # ------------------------------------------------------------------ #
    # Additional: Multi-color detection with voting
    # ------------------------------------------------------------------ #
    def detect_dominant_color(self, masks):
        """Detect the color with the largest contour across all target colors.

        Also accumulates color votes over multiple frames for robust classification
        (as done in ColorSorting/Palletizing).

        Args:
            masks: Dict from segment_colors().

        Returns:
            (color_name, contour, area) for the dominant color.
            color_name is None if nothing detected above min_detect_area.
        """
        best_color = None
        best_contour = None
        best_area = 0

        for color_name, mask in masks.items():
            contour, area = self.find_largest_contour(mask)
            if contour is not None and area > best_area:
                best_area = area
                best_color = color_name
                best_contour = contour

        if best_area <= self.min_detect_area:
            return None, None, 0

        # Color voting for robust classification
        if self.color_vote_count > 0:
            color_code = {'red': 1, 'green': 2, 'blue': 3}.get(best_color, 0)
            self._color_votes.append(color_code)

            if len(self._color_votes) >= self.color_vote_count:
                voted = int(round(np.mean(np.array(self._color_votes))))
                self._color_votes = []
                voted_color = {1: 'red', 2: 'green', 3: 'blue'}.get(voted)
                if voted_color is not None:
                    best_color = voted_color

        return best_color, best_contour, best_area

    # ------------------------------------------------------------------ #
    # Convenience: full pipeline in one call
    # ------------------------------------------------------------------ #
    def process_frame(self, img):
        """Run the full perception pipeline on a single frame.

        Args:
            img: Raw BGR camera frame.

        Returns:
            dict with detection results, or None if no block detected.
            Keys: 'color', 'world_x', 'world_y', 'rotation_angle',
                  'box', 'stable_position' (tuple or None),
                  'annotated_img'.
        """
        preprocessed = self.preprocess(img.copy())
        masks = self.segment_colors(preprocessed)

        # Use multi-color detection if multiple targets, else single-color
        if len(self.target_colors) > 1:
            color_name, contour, area = self.detect_dominant_color(masks)
        else:
            color_name = self.target_colors[0]
            mask = masks.get(color_name)
            if mask is None:
                return None
            contour, area = self.find_largest_contour(mask)
            if area <= self.min_detect_area:
                contour = None

        if contour is None:
            self._use_roi = False
            return None

        pose = self.get_block_pose(contour)
        if pose is None:
            return None

        stable = self.stabilize_detection(pose['world_x'], pose['world_y'])

        annotated = self.annotate_frame(
            img, pose['box'],
            pose['world_x'], pose['world_y'],
            color_name,
            label_text=f"Color: {color_name}",
        )

        return {
            'color': color_name,
            'world_x': pose['world_x'],
            'world_y': pose['world_y'],
            'rotation_angle': pose['rotation_angle'],
            'box': pose['box'],
            'stable_position': stable,
            'annotated_img': annotated,
        }
