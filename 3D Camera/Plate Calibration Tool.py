"""
Plate calibration utility for the RealSense camera setup.

Click the four plate corners in order, save the geometry calibration, and
optionally capture an empty-plate depth reference for later depth correction.
"""

import json

import cv2
import numpy as np
import pyrealsense2 as rs

# ================= CONFIG =================
WIDTH = 640
HEIGHT = 480
FPS_COLOR = 60
FPS_DEPTH = 90

ANGLE = 0
CENTER = (WIDTH // 2, HEIGHT // 2)
ROT_MAT = cv2.getRotationMatrix2D(CENTER, ANGLE, 1.0)

WINDOW_NAME = 'Plate Calibration'
OUTPUT_FILE = 'plate_calibration.json'
DEPTH_REFERENCE_FILE = 'plate_depth_reference.npy'

PLATE_SIZE_MM = 400
DEPTH_FRAMES_TO_AVERAGE = 100
MASK_MARGIN = 10

# Intel RealSense temporal filter settings for the empty-plate depth capture.
TEMPORAL_ALPHA = 0.4
TEMPORAL_DELTA = 20

clicked_points = []


# ================= MOUSE CALLBACK =================
def mouse_callback(event, x, y, flags, param):
    """Store clicked plate corners in the required TL -> TR -> BR -> BL order."""
    global clicked_points

    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append((x, y))
        print(f'Point {len(clicked_points)}: ({x}, {y})')


# ================= DRAW OVERLAY =================
def draw_overlay(frame):
    """Draw selected corners, plate outline, and keyboard instructions."""
    display = frame.copy()
    labels = ['TL', 'TR', 'BR', 'BL']

    for i, point in enumerate(clicked_points):
        cv2.circle(display, point, 6, (0, 0, 255), -1)
        cv2.putText(
            display,
            f'{i + 1}:{labels[i]}',
            (point[0] + 10, point[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2
        )

    # Connect the clicked points so the user can immediately see the plate order.
    if len(clicked_points) >= 2:
        for i in range(len(clicked_points) - 1):
            cv2.line(display, clicked_points[i], clicked_points[i + 1], (255, 0, 255), 2)

    if len(clicked_points) == 4:
        cv2.line(display, clicked_points[3], clicked_points[0], (255, 0, 255), 2)

        xs = [p[0] for p in clicked_points]
        ys = [p[1] for p in clicked_points]
        cv2.rectangle(display, (min(xs), min(ys)), (max(xs), max(ys)), (0, 255, 0), 2)

    instructions = [
        'Click corners: TL -> TR -> BR -> BL',
        'ENTER = Save geometry',
        'D = Capture depth reference',
        'R = Reset',
        'ESC = Quit'
    ]

    y_text = 25
    for line in instructions:
        cv2.putText(display, line, (15, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        y_text += 25

    return display


# ================= SAVE GEOMETRY =================
def save_geometry():
    """Save clicked plate corners and the bounding box used by runtime scripts."""
    xs = [p[0] for p in clicked_points]
    ys = [p[1] for p in clicked_points]

    calibration_data = {
        'plate_size_mm': PLATE_SIZE_MM,
        'top_left': clicked_points[0],
        'top_right': clicked_points[1],
        'bottom_right': clicked_points[2],
        'bottom_left': clicked_points[3],
        'plate_quad': clicked_points.copy(),
        'plate_x1': min(xs),
        'plate_x2': max(xs),
        'plate_y1': min(ys),
        'plate_y2': max(ys),
        'depth_reference_file': DEPTH_REFERENCE_FILE
    }

    with open(OUTPUT_FILE, 'w') as f:
        json.dump(calibration_data, f, indent=4)

    print('\nGeometry calibration saved:')
    print(OUTPUT_FILE)

    return calibration_data


# ================= DEPTH CALIBRATION =================
def build_plate_mask():
    """Create an eroded mask so the depth reference only contains the plate interior."""
    plate_mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    quad = np.array(clicked_points, dtype=np.int32)

    cv2.fillPoly(plate_mask, [quad], 255)

    # Remove the edge region because plate boundaries often contain depth artifacts.
    kernel = np.ones((2 * MASK_MARGIN + 1, 2 * MASK_MARGIN + 1), np.uint8)
    return cv2.erode(plate_mask, kernel, iterations=1)


def capture_depth_reference():
    """Average multiple empty-plate depth frames and save the masked result."""
    if len(clicked_points) != 4:
        print('Please calibrate plate corners first.')
        return

    print('\n==============================')
    print('DEPTH REFERENCE CALIBRATION')
    print('Make sure the plate is EMPTY.')
    print('Capturing depth reference...')
    print('==============================')

    temporal = rs.temporal_filter()
    temporal.set_option(rs.option.filter_smooth_alpha, TEMPORAL_ALPHA)
    temporal.set_option(rs.option.filter_smooth_delta, TEMPORAL_DELTA)

    depth_sum = None
    valid_frame_count = 0

    for i in range(DEPTH_FRAMES_TO_AVERAGE):
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        depth_frame = temporal.process(depth_frame)
        depth = np.asanyarray(depth_frame.get_data()).astype(np.float32)

        # Convert RealSense raw depth units to millimeters.
        depth *= depth_scale * 1000.0

        # Apply the same image transform as the runtime tracking code.
        depth = cv2.warpAffine(depth, ROT_MAT, (WIDTH, HEIGHT))
        depth = cv2.flip(depth, -1)

        if depth_sum is None:
            depth_sum = depth
        else:
            depth_sum += depth

        valid_frame_count += 1

        if (i + 1) % 10 == 0:
            print(f'Captured {i + 1}/{DEPTH_FRAMES_TO_AVERAGE}')

    if depth_sum is None or valid_frame_count == 0:
        print('No valid depth frames captured.')
        return

    depth_reference = depth_sum / valid_frame_count
    plate_mask = build_plate_mask()

    depth_reference[plate_mask == 0] = 0
    np.save(DEPTH_REFERENCE_FILE, depth_reference)

    valid_depth_values = depth_reference[depth_reference > 0]
    mean_depth = np.mean(valid_depth_values) if valid_depth_values.size else 0.0

    print('\nDepth reference saved:')
    print(DEPTH_REFERENCE_FILE)
    print(f'Mean plate depth: {mean_depth:.1f} mm')


# ================= CAMERA INIT =================
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS_COLOR)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS_DEPTH)

profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
align = rs.align(rs.stream.color)

cv2.namedWindow(WINDOW_NAME)
cv2.setMouseCallback(WINDOW_NAME, mouse_callback)


# ================= MAIN LOOP =================
try:
    while True:
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)

        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        frame = cv2.warpAffine(frame, ROT_MAT, (WIDTH, HEIGHT))
        frame = cv2.flip(frame, -1)

        display = draw_overlay(frame)
        cv2.imshow(WINDOW_NAME, display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            clicked_points = []
            print('Calibration points reset.')

        elif key == 13:
            if len(clicked_points) == 4:
                save_geometry()
            else:
                print('Need exactly 4 points before saving.')

        elif key == ord('d'):
            if len(clicked_points) != 4:
                print('Please define the 4 corners first.')
                continue

            save_geometry()
            capture_depth_reference()

        elif key == 27:
            print('Calibration cancelled.')
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
