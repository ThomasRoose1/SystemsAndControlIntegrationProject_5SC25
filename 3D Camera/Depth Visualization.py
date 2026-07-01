import os
import cv2
import numpy as np
import pyrealsense2 as rs
import math
from collections import deque
import json

# ============================================
# CAMERA PARAMETERS
# ============================================

WIDTH = 640
HEIGHT = 480

FPS_COLOR = 60
FPS_DEPTH = 90

WINDOW_W = 640
WINDOW_H = 480

SAVE_FOLDER = r"C:\git\SystemsAndControlIntegrationProject_5SC25\3D Camera\Depth Figures"

os.makedirs(SAVE_FOLDER, exist_ok=True)

# ============================================
# DETECTION PARAMETERS
# ============================================

THRESHOLD = 95

MIN_AREA = 200
MAX_AREA = 2000

MIN_CIRCULARITY = 0.85

DEPTH_MIN_MM = 400
DEPTH_MAX_MM = 900

DEPTH_ROI_RADIUS = 5

# ================= LOAD CALIBRATION =================
# Calibration parameters
CALIBRATION_FILE = 'plate_calibration.json'
PLATE_SIZE_MM = 390.0
GRID_SPACING_MM = 100.0

# The calibration file stores the four detected plate corners in image pixels.
with open(CALIBRATION_FILE, 'r') as f:
    calib = json.load(f)

plate_quad = np.array(calib['plate_quad'], dtype=np.float32)
TL, TR, BR, BL = plate_quad

# ================= HOMOGRAPHY =================
# Homography maps camera pixels on the plate plane to real plate coordinates.
image_points = np.array([TL, TR, BR, BL], dtype=np.float32)
half = PLATE_SIZE_MM / 2
world_points = np.array(
    [[-half, -half], [half, -half], [half, half], [-half, half]],
    dtype=np.float32
)
H, _ = cv2.findHomography(image_points, world_points)

# Estimate the plate rotation from all four edges so the image can be leveled.
top_angle = math.degrees(math.atan2(TR[1] - TL[1], TR[0] - TL[0]))
bottom_angle = math.degrees(math.atan2(BR[1] - BL[1], BR[0] - BL[0]))

left_angle = math.degrees(math.atan2(BL[1] - TL[1], BL[0] - TL[0])) - 90.0
right_angle = math.degrees(math.atan2(BR[1] - TR[1], BR[0] - TR[0])) - 90.0

angle_deg = np.mean([top_angle, bottom_angle, left_angle, right_angle])

print()
print('===== PLATE ANGLE ESTIMATION =====')
print(f'Top    : {top_angle:.3f}')
print(f'Bottom : {bottom_angle:.3f}')
print(f'Left   : {left_angle:.3f}')
print(f'Right  : {right_angle:.3f}')
print(f'Final  : {angle_deg:.3f}')
print('==================================')

CENTER = (WIDTH // 2, HEIGHT // 2)
ROT_MAT = cv2.getRotationMatrix2D(CENTER, angle_deg, 1.0)

# ============================================
# FILTER PARAMETERS
# ============================================

TEMPORAL_ALPHA = 0.6
TEMPORAL_DELTA = 20

SPATIAL_MAGNITUDE = 2
SPATIAL_ALPHA = 0.5
SPATIAL_DELTA = 5

DEPTH_OUTLIER_SIGMA = 2.0

USE_ROBUST_DEPTH = True

USE_SPATIAL_FILTER = True

Z_FILTER_ALPHA = 0.5

# ============================================

freeze = False

saved_counter = 1

filtered_z = None

# ============================================
# REALSENSE INITIALIZATION
# ============================================

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    WIDTH,
    HEIGHT,
    rs.format.bgr8,
    FPS_COLOR
)

config.enable_stream(
    rs.stream.depth,
    WIDTH,
    HEIGHT,
    rs.format.z16,
    FPS_DEPTH
)

profile = pipeline.start(config)

# ============================================
# DEPTH SCALE
# ============================================

depth_sensor = profile.get_device().first_depth_sensor()

depth_scale = depth_sensor.get_depth_scale()

print(f"Depth scale = {depth_scale:.6f} m")

# ============================================
# ALIGN DEPTH TO COLOR
# ============================================

align = rs.align(rs.stream.color)

# ============================================
# DEPTH FILTERS
# ============================================

spatial = rs.spatial_filter()

spatial.set_option(
    rs.option.filter_magnitude,
    SPATIAL_MAGNITUDE
)

spatial.set_option(
    rs.option.filter_smooth_alpha,
    SPATIAL_ALPHA
)

spatial.set_option(
    rs.option.filter_smooth_delta,
    SPATIAL_DELTA
)

temporal = rs.temporal_filter()

temporal.set_option(
    rs.option.filter_smooth_alpha,
    TEMPORAL_ALPHA
)

temporal.set_option(
    rs.option.filter_smooth_delta,
    TEMPORAL_DELTA
)

hole_filling = rs.hole_filling_filter()

# ============================================
# DEPTH COLORIZER
# ============================================

colorizer = rs.colorizer()

colorizer.set_option(
    rs.option.color_scheme,
    2
)

# ============================================
# AUTO EXPOSURE WARM-UP
# ============================================

print("Warming up camera...")

for _ in range(30):

    pipeline.wait_for_frames()

print("Camera ready.")

# ============================================
# HELPER FUNCTIONS
# ============================================

def contour_area(contour):
    return cv2.contourArea(contour)

# ============================================================
# DRAW DEPTH LEGEND
# ============================================================

def draw_depth_legend(depth_img):

    h = depth_img.shape[0]

    x0 = depth_img.shape[1] - 50
    y0 = 20
    y1 = h - 20

    for y in range(y0, y1):

        ratio = (y - y0) / max((y1 - y0), 1)

        depth_mm = DEPTH_MIN_MM + ratio * (DEPTH_MAX_MM - DEPTH_MIN_MM)

        norm = int(
            255 *
            (depth_mm - DEPTH_MIN_MM) /
            (DEPTH_MAX_MM - DEPTH_MIN_MM)
        )

        color = cv2.applyColorMap(
            np.uint8([[norm]]),
            cv2.COLORMAP_JET
        )[0][0]

        cv2.line(
            depth_img,
            (x0, y),
            (x0 + 25, y),
            tuple(int(c) for c in color),
            1
        )

    cv2.rectangle(
        depth_img,
        (x0, y0),
        (x0 + 25, y1),
        (255,255,255),
        1
    )

    cv2.putText(
        depth_img,
        f"{DEPTH_MIN_MM}",
        (x0 - 65, y1 + 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255,255,255),
        1
    )

    cv2.putText(
        depth_img,
        f"{DEPTH_MAX_MM}",
        (x0 - 65, y0 + 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255,255,255),
        1
    )

    cv2.putText(
        depth_img,
        "mm",
        (x0 - 30, y0 - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255,255,255),
        1
    )

def contour_perimeter(contour):
    return cv2.arcLength(contour, True)


def contour_circularity(contour):

    area = contour_area(contour)

    perimeter = contour_perimeter(contour)

    if perimeter == 0:
        return 0

    return 4 * np.pi * area / (perimeter * perimeter)


def contour_centroid(contour):

    M = cv2.moments(contour)

    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    return (cx, cy)


# ============================================================
# BALL DETECTION
# ============================================================

def detect_ball(frame):

    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

    blurred = cv2.GaussianBlur(
        gray,
        (5,5),
        0
    )

    _, thresh = cv2.threshold(
        blurred,
        THRESHOLD,
        255,
        cv2.THRESH_BINARY_INV
    )

    kernel = np.ones(
        (5,5),
        np.uint8
    )

    mask = cv2.morphologyEx(
        thresh,
        cv2.MORPH_OPEN,
        kernel
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    candidates = []

    for contour in contours:

        area = contour_area(contour)

        if area < MIN_AREA or area > MAX_AREA:
            continue

        circ = contour_circularity(contour)

        if circ < MIN_CIRCULARITY:
            continue

        (x, y), radius = cv2.minEnclosingCircle(contour)

        candidates.append(
            (
                contour,
                (x, y),
                radius
            )
        )

    if len(candidates) == 0:

        return None, gray, blurred, thresh, mask

    contour, center, radius = max(
        candidates,
        key=lambda c: cv2.contourArea(c[0])
    )

    return (
        (
            contour,
            center,
            radius
        ),
        gray,
        blurred,
        thresh,
        mask
    )


# ============================================================
# DEPTH ROI
# ============================================================

def get_depth_roi(depth_image, cx, cy):

    r = DEPTH_ROI_RADIUS

    x1 = max(0, cx-r)
    x2 = min(WIDTH, cx+r+1)

    y1 = max(0, cy-r)
    y2 = min(HEIGHT, cy+r+1)

    return depth_image[y1:y2, x1:x2], x1, y1


# ============================================================
# ROBUST DEPTH ESTIMATION
# ============================================================

def robust_depth_estimate(depth_roi):

    valid = depth_roi[
        (depth_roi > DEPTH_MIN_MM) &
        (depth_roi < DEPTH_MAX_MM)
    ]

    if len(valid) == 0:

        return None, valid

    median = np.median(valid)

    sigma = np.std(valid)

    if sigma < 1:

        sigma = 1

    robust = valid[
        np.abs(valid - median)
        < DEPTH_OUTLIER_SIGMA * sigma
    ]

    if len(robust) == 0:

        return median, valid

    return np.mean(robust), robust


# ============================================================
# SIMPLE EMA FILTER
# ============================================================

def ema_filter(z):

    global filtered_z

    if z is None:
        return filtered_z

    if filtered_z is None:

        filtered_z = z

    else:

        filtered_z = (
            Z_FILTER_ALPHA * z +
            (1-Z_FILTER_ALPHA) * filtered_z
        )

    return filtered_z


# ============================================================
# DRAW DEPTH ROI
# ============================================================

def draw_depth_roi(img, cx, cy):

    r = DEPTH_ROI_RADIUS

    cv2.rectangle(
        img,
        (cx-r, cy-r),
        (cx+r, cy+r),
        (255,255,255),
        2
    )

    cv2.circle(
        img,
        (cx, cy),
        4,
        (0,0,255),
        -1
    )


# ============================================================
# DRAW VALID DEPTH PIXELS
# ============================================================

def draw_valid_pixels(img, depth_image, cx, cy):

    r = DEPTH_ROI_RADIUS

    for yy in range(-r, r+1):

        for xx in range(-r, r+1):

            px = cx + xx
            py = cy + yy

            if px < 0 or py < 0:
                continue

            if px >= WIDTH or py >= HEIGHT:
                continue

            d = depth_image[py, px]

            if DEPTH_MIN_MM < d < DEPTH_MAX_MM:

                cv2.circle(
                    img,
                    (px, py),
                    1,
                    (0,255,0),
                    -1
                )

            else:

                cv2.circle(
                    img,
                    (px, py),
                    1,
                    (0,0,255),
                    -1
                )

# ============================================
# MAIN LOOP
# ============================================

while True:

    # --------------------------------------------------
    # Freeze support
    # --------------------------------------------------

    if not freeze:

        # ===============================================
        # GET FRAMES
        # ===============================================

        frames = pipeline.wait_for_frames()

        frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # ===============================================
        # APPLY REALSENSE FILTERS
        # ===============================================

        if USE_SPATIAL_FILTER:
            depth_frame = spatial.process(depth_frame)

        depth_frame = temporal.process(depth_frame)

        depth_frame = hole_filling.process(depth_frame)

        # ===============================================
        # CONVERT TO NUMPY
        # ===============================================

        frame = np.asanyarray(
            color_frame.get_data()
        )

        depth_image = np.asanyarray(
            depth_frame.get_data()
        )

        # Same orientation as Phase_G3

        frame = cv2.warpAffine(
            frame,
            ROT_MAT,
            (WIDTH,HEIGHT)
        )

        depth_image = cv2.warpAffine(
            depth_image,
            ROT_MAT,
            (WIDTH,HEIGHT)
        )

        frame = cv2.flip(frame,-1)
        depth_image = cv2.flip(depth_image,-1)

        # ===============================================
        # DEPTH COLORMAP
        # ===============================================

        # Convert depth to millimetres
        depth_mm = depth_image.astype(np.float32)

        # Normalize only within the useful range
        depth_norm = np.clip(
            (depth_mm - DEPTH_MIN_MM) /
            (DEPTH_MAX_MM - DEPTH_MIN_MM),
            0,
            1
        )

        depth_uint8 = (255 * (1 - depth_norm)).astype(np.uint8)

        # Apply OpenCV colormap
        depth_vis = cv2.applyColorMap(
            depth_uint8,
            cv2.COLORMAP_JET
        )

        # ===============================================
        # BALL DETECTION
        # ===============================================

        detection, gray, blurred, thresh, mask = detect_ball(frame)

        # Default images

        raw_depth_img = depth_vis.copy()
        roi_depth_img = depth_vis.copy()

        final_img = frame.copy()

        raw_depth = None
        robust_depth = None
        filtered_depth = None

        valid_pixels = 0

        # ===============================================
        # IF BALL FOUND
        # ===============================================

        if detection is not None:

            contour, center, radius = detection

            cx = int(center[0])
            cy = int(center[1])

            # ------------------------------------------
            # Draw ROI
            # ------------------------------------------

            draw_depth_roi(
                raw_depth_img,
                cx,
                cy
            )
            draw_depth_legend(raw_depth_img)

            draw_depth_roi(
                roi_depth_img,
                cx,
                cy
            )
            draw_depth_legend(roi_depth_img)

            # ------------------------------------------
            # Extract ROI
            # ------------------------------------------

            roi, x0, y0 = get_depth_roi(
                depth_image,
                cx,
                cy
            )

            # ------------------------------------------
            # Raw center depth
            # ------------------------------------------

            raw_depth = depth_image[
                cy,
                cx
            ]

            # ------------------------------------------
            # Robust depth estimate
            # ------------------------------------------

            robust_depth, valid = robust_depth_estimate(
                roi
            )

            valid_pixels = len(valid)

            # ------------------------------------------
            # Temporal / EMA filtering
            # ------------------------------------------

            filtered_depth = ema_filter(
                robust_depth
            )

            # ------------------------------------------
            # Draw valid depth pixels
            # ------------------------------------------

            draw_valid_pixels(
                roi_depth_img,
                depth_image,
                cx,
                cy
            )

            # ------------------------------------------
            # Final RGB visualization
            # ------------------------------------------

            cv2.circle(
                final_img,
                (cx, cy),
                int(radius),
                (255, 0, 255),
                2
            )

            cv2.circle(
                final_img,
                (cx, cy),
                4,
                (0, 0, 255),
                -1
            )

            # ------------------------------------------
            # Text overlays
            # ------------------------------------------

            if raw_depth is not None:

                cv2.putText(
                    raw_depth_img,
                    f"Raw Depth : {raw_depth:.0f} mm",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255,255,255),
                    2
                )

            if robust_depth is not None:

                cv2.putText(
                    roi_depth_img,
                    f"Robust Depth : {robust_depth:.1f} mm",
                    (20,35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255,255,255),
                    2
                )

                cv2.putText(
                    roi_depth_img,
                    f"Valid Pixels : {valid_pixels}",
                    (20,65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255,255,255),
                    2
                )

            if filtered_depth is not None:

                cv2.putText(
                    final_img,
                    f"Final Z : {filtered_depth:.1f} mm",
                    (20,35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0,255,0),
                    2
                )

                cv2.putText(
                    final_img,
                    f"Raw : {raw_depth:.1f} mm",
                    (20,70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255,255,255),
                    2
                )

                cv2.putText(
                    final_img,
                    f"Robust : {robust_depth:.1f} mm",
                    (20,100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255,255,255),
                    2
                )

                cv2.putText(
                    final_img,
                    f"EMA : {filtered_depth:.1f} mm",
                    (20,130),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0,255,0),
                    2
                )

    # =====================================================
    # DISPLAY WINDOWS
    # =====================================================

    cv2.imshow(
        "1 Raw Depth Frame",
        raw_depth_img
    )

    cv2.imshow(
        "2 Depth ROI Averaging",
        roi_depth_img
    )

    # -----------------------------------------------------

    temporal_img = np.zeros(
        (HEIGHT, WIDTH, 3),
        dtype=np.uint8
    )

    cv2.putText(
        temporal_img,
        "Temporal Filtering",
        (20,40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255,255,255),
        2
    )

    if raw_depth is not None:

        cv2.putText(
            temporal_img,
            f"Raw Center Pixel : {raw_depth:.1f} mm",
            (20,110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255,255,255),
            2
        )

    if robust_depth is not None:

        cv2.putText(
            temporal_img,
            f"ROI Robust Mean : {robust_depth:.1f} mm",
            (20,170),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0,255,255),
            2
        )

    if filtered_depth is not None:

        cv2.putText(
            temporal_img,
            f"Adaptive EMA : {filtered_depth:.1f} mm",
            (20,230),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0,255,0),
            2
        )

        cv2.putText(
            temporal_img,
            f"Alpha = {Z_FILTER_ALPHA:.2f}",
            (20,290),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (180,180,180),
            2
        )

    cv2.imshow(
        "3 Temporal Filtering",
        temporal_img
    )

    cv2.imshow(
        "4 Final Z Measurement",
        final_img
    )

    # =====================================================
    # KEYBOARD
    # =====================================================

    key = cv2.waitKey(1) & 0xFF

    if key == ord('f'):

        freeze = not freeze

        if freeze:
            print("Frame frozen.")
        else:
            print("Live mode.")

    # -----------------------------------------------------

    elif key == ord('s'):

        print("Saving figures...")

        cv2.imwrite(
            os.path.join(
                SAVE_FOLDER,
                "1_RawDepth.png"
            ),
            raw_depth_img
        )

        cv2.imwrite(
            os.path.join(
                SAVE_FOLDER,
                "2_DepthROI.png"
            ),
            roi_depth_img
        )

        cv2.imwrite(
            os.path.join(
                SAVE_FOLDER,
                "3_TemporalFiltering.png"
            ),
            temporal_img
        )

        cv2.imwrite(
            os.path.join(
                SAVE_FOLDER,
                "4_FinalDepthMeasurement.png"
            ),
            final_img
        )

        print("Saved.")

    # -----------------------------------------------------

    elif key == 27:

        break


# =====================================================
# CLEANUP
# =====================================================

pipeline.stop()

cv2.destroyAllWindows()