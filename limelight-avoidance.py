"""Limelight SnapScript for tracking yellow cargo and avoiding red/blue robots.

This script is intended for Limelight 3+/AprilTag firmware running the Python
SnapScript runtime. It exposes a single `runPipeline(image, llrobot)` function
that Limelight calls for each frame. The script returns both the annotated
image and a dictionary of targeting/avoidance metadata that can be consumed via
network tables (`llpython`).

Outputs:
	{
		"hasBall": bool,
		"ballCenterNorm": (float, float),  # normalized 0..1 image coords
		"ballAreaNorm": float,             # area relative to frame area
		"ballBoundingBox": [x, y, w, h],   # pixel rect or None
		"avoidanceVector": (float, float), # suggested strafe/drive offset
		"danger": bool,                    # true when robot detected in front
		"redRobots": [ {"center": (x,y), "area": a}, ... ],
		"blueRobots": [ ... ]
	}

Tuning Tips:
	- Adjust HSV thresholds below to match the lighting conditions of your
	  field/practice space.
	- `MIN_BALL_AREA` rejects noise; set lower for distant cargo.
	- `BUMPER_BLOCK_THRESHOLD` determines when another robot is considered to
	  be blocking the shot path (percentage of image width).
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

import cv2
import numpy as np


# Yellow cargo color thresholds (HSV)
YELLOW_LOWER = np.array([20, 120, 120])
YELLOW_UPPER = np.array([35, 255, 255])

# Blue bumper thresholds (HSV). Narrow saturation range reduces field glare.
BLUE_LOWER = np.array([95, 120, 60])
BLUE_UPPER = np.array([125, 255, 255])

# Red uses two ranges because hue wraps around at 180.
RED_LOWER_1 = np.array([0, 140, 90])
RED_UPPER_1 = np.array([10, 255, 255])
RED_LOWER_2 = np.array([170, 140, 90])
RED_UPPER_2 = np.array([180, 255, 255])

MIN_BALL_AREA = 200  # pixels
MIN_ROBOT_AREA = 400  # pixels

# When any robot bounding box occupies this fraction of the frame width, flag
# danger (i.e., they're directly ahead of us).
BUMPER_BLOCK_THRESHOLD = 0.28


def _mask_and_clean(hsv_image: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
	mask = cv2.inRange(hsv_image, lower, upper)
	kernel = np.ones((5, 5), np.uint8)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
	return mask


def _detect_colored_regions(mask: np.ndarray, min_area: int) -> List[Tuple[np.ndarray, float, Tuple[int, int, int, int]]]:
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	detections = []
	for contour in contours:
		area = cv2.contourArea(contour)
		if area < min_area:
			continue
		x, y, w, h = cv2.boundingRect(contour)
		detections.append((contour, area, (x, y, w, h)))
	return detections


def _pick_largest(detections):
	if not detections:
		return None
	return max(detections, key=lambda det: det[1])


def _norm_center(rect: Tuple[int, int, int, int], width: int, height: int) -> Tuple[float, float]:
	x, y, w, h = rect
	cx = (x + w / 2) / width
	cy = (y + h / 2) / height
	return cx, cy


def runPipeline(image, llrobot):
	"""Entrypoint called by Limelight every frame."""

	if image is None or image.size == 0:
		return image, {
			"hasBall": False,
			"ballCenterNorm": (0.0, 0.0),
			"ballAreaNorm": 0.0,
			"ballBoundingBox": None,
			"avoidanceVector": (0.0, 0.0),
			"danger": False,
			"redRobots": [],
			"blueRobots": [],
		}

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	height, width = hsv.shape[:2]
	frame_area = float(width * height)

	# --- Yellow cargo detection ---
	yellow_mask = _mask_and_clean(hsv, YELLOW_LOWER, YELLOW_UPPER)
	yellow_detections = _detect_colored_regions(yellow_mask, MIN_BALL_AREA)
	best_ball = _pick_largest(yellow_detections)

	has_ball = best_ball is not None
	ball_center_norm = (0.0, 0.0)
	ball_area_norm = 0.0
	ball_rect = None

	if has_ball:
		_, area, rect = best_ball
		ball_rect = [int(v) for v in rect]
		ball_center_norm = _norm_center(rect, width, height)
		ball_area_norm = float(area) / frame_area
		cv2.rectangle(image, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 255), 2)
		cv2.circle(
			image,
			(int(ball_center_norm[0] * width), int(ball_center_norm[1] * height)),
			5,
			(0, 255, 255),
			-1,
		)

	# --- Robot avoidance (red + blue bumpers) ---
	blue_mask = _mask_and_clean(hsv, BLUE_LOWER, BLUE_UPPER)
	blue_detections = _detect_colored_regions(blue_mask, MIN_ROBOT_AREA)

	red_mask1 = _mask_and_clean(hsv, RED_LOWER_1, RED_UPPER_1)
	red_mask2 = _mask_and_clean(hsv, RED_LOWER_2, RED_UPPER_2)
	red_mask = cv2.bitwise_or(red_mask1, red_mask2)
	red_detections = _detect_colored_regions(red_mask, MIN_ROBOT_AREA)

	def pack_detections(detections, color):
		packed = []
		for _, area, rect in detections:
			cx, cy = _norm_center(rect, width, height)
			packed.append({"center": (cx, cy), "area": area / frame_area, "rect": [int(v) for v in rect]})
			cv2.rectangle(image, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), color, 2)
		return packed

	red_info = pack_detections(red_detections, (0, 0, 255))
	blue_info = pack_detections(blue_detections, (255, 0, 0))

	# Build avoidance vector by summing repulsive forces from detected robots.
	avoid_x = 0.0
	avoid_y = 0.0
	danger = False

	for robots in (red_info, blue_info):
		for det in robots:
			cx, cy = det["center"]
			rect_w = det["rect"][2] / width
			if rect_w > BUMPER_BLOCK_THRESHOLD and cy > 0.4:
				danger = True

			# Simple inverse-square repulsion along X, encourage backing up if close.
			dx = 0.5 - cx
			dy = 0.7 - cy  # focus on lower part of frame (closer to robot)
			distance = max(math.hypot(dx, dy), 1e-3)
			strength = det["area"] * 3.0
			avoid_x += (dx / distance) * strength
			avoid_y += (dy / distance) * strength

	# Optional: combine avoidance with chase behavior to produce a point steer command.
	avoidance_vector = (float(avoid_x), float(avoid_y))

	overlay_text = f"Ball:{int(has_ball)} Danger:{int(danger)}"
	cv2.putText(image, overlay_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

	result: Dict[str, object] = {
		"hasBall": has_ball,
		"ballCenterNorm": ball_center_norm,
		"ballAreaNorm": ball_area_norm,
		"ballBoundingBox": ball_rect,
		"avoidanceVector": avoidance_vector,
		"danger": danger,
		"redRobots": red_info,
		"blueRobots": blue_info,
	}

	return image, result

