import socket
import struct
import cv2
import numpy as np
import pyrealsense2 as rs
import math
import json
import time

# ================= CONFIG =================
WIDTH = 640
HEIGHT = 480
FPS = 60
FPS_Depth = 90
CALIBRATION_FILE = 'plate_calibration.json'
MASK_MARGIN = 15   # pixels

SHOW_MASK = True
SHOW_DEPTH_VIEW = False
DEPTH_MIN_MM = 400
DEPTH_MAX_MM = 900

# 2D Parameters
THRESHOLD = 75
MIN_AREA = 200
MAX_AREA = 2000
MIN_CIRCULARITY = 0.75
MAX_TRACK_DISTANCE = 120

# Depth Parameters
TEMPORAL_ALPHA = 0.4
TEMPORAL_DELTA = 20
DEPTH_CALIBRATION = False
from collections import deque

Z_STATS_WINDOW = 300
z_history = deque(maxlen=Z_STATS_WINDOW)

PLATE_SIZE_MM = 400.0
GRID_SPACING_MM = 100.0

# Improved depth measurement
DEPTH_ROI_RADIUS = 5   # 11x11 median ROI

# UDP
serverAddressPort = ("192.168.140.8", 49001)
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
DEBUG_PRINT_UDP = False

# ================= LOAD CALIBRATION =================
with open(CALIBRATION_FILE, 'r') as f:
    calib = json.load(f)
    DEPTH_REFERENCE_FILE = calib['depth_reference_file']

    plate_depth_reference = np.load(
        DEPTH_REFERENCE_FILE
    )

    print(
        f'Loaded depth reference: '
        f'{DEPTH_REFERENCE_FILE}'
    )

plate_quad = np.array(calib['plate_quad'], dtype=np.float32)
TL, TR, BR, BL = plate_quad

edge = TR - TL
angle_deg = math.degrees(math.atan2(edge[1], edge[0]))
print(f'Computed plate angle: {angle_deg:.2f} deg')

CENTER = (WIDTH // 2, HEIGHT // 2)
ROT_MAT = cv2.getRotationMatrix2D(CENTER, -angle_deg, 1.0)

ones = np.ones((4, 1))
plate_h = np.hstack([plate_quad, ones])
rotated_quad = (ROT_MAT @ plate_h.T).T.astype(np.int32)

PLATE_X1 = int(np.min(rotated_quad[:, 0]))
PLATE_X2 = int(np.max(rotated_quad[:, 0]))
PLATE_Y1 = int(np.min(rotated_quad[:, 1]))
PLATE_Y2 = int(np.max(rotated_quad[:, 1]))

PLATE_CENTER_X = int((PLATE_X1 + PLATE_X2) / 2)
PLATE_CENTER_Y = int((PLATE_Y1 + PLATE_Y2) / 2)

SCALE_X = PLATE_SIZE_MM / max((PLATE_X2 - PLATE_X1), 1)
SCALE_Y = PLATE_SIZE_MM / max((PLATE_Y2 - PLATE_Y1), 1)

# ================= HELPERS =================
def pixel_to_control_coords(pixel_xy):
    x_mm = (pixel_xy[0] - PLATE_CENTER_X) * SCALE_X
    y_mm = (PLATE_CENTER_Y - pixel_xy[1]) * SCALE_Y
    return x_mm, y_mm


def send_ball_position_xyz_mm(ctrl_xy, z_value, detected_flag):
    x_mm = int(ctrl_xy[0])
    y_mm = int(ctrl_xy[1])
    z_mm = int(z_value)

    x_bytes = x_mm.to_bytes(2, byteorder='little', signed=True)
    y_bytes = y_mm.to_bytes(2, byteorder='little', signed=True)
    z_bytes = z_mm.to_bytes(2, byteorder='little', signed=True)

    packet = bytearray([
        x_bytes[0], x_bytes[1],
        0, 0,
        y_bytes[0], y_bytes[1],
        0, 0,
        z_bytes[0], z_bytes[1],
        detected_flag
    ])
    if DEBUG_PRINT_UDP:
        print(f"UDP -> X={x_mm}, Y={y_mm}, Z={z_mm}, Flag={detected_flag}")
    UDPClientSocket.sendto(packet, serverAddressPort)

def get_plate_reference_mm(
        depth_reference,
        x,
        y):

    x1 = max(0, x - DEPTH_ROI_RADIUS)
    x2 = min(
        depth_reference.shape[1],
        x + DEPTH_ROI_RADIUS + 1
    )

    y1 = max(0, y - DEPTH_ROI_RADIUS)
    y2 = min(
        depth_reference.shape[0],
        y + DEPTH_ROI_RADIUS + 1
    )

    roi = depth_reference[
        y1:y2,
        x1:x2
    ]

    valid = roi[roi > 0]

    if valid.size == 0:
        return None

    return np.median(valid)

def get_median_depth_mm(depth_raw, x, y, depth_scale):
    x1 = max(0, x - DEPTH_ROI_RADIUS)
    x2 = min(depth_raw.shape[1], x + DEPTH_ROI_RADIUS + 1)
    y1 = max(0, y - DEPTH_ROI_RADIUS)
    y2 = min(depth_raw.shape[0], y + DEPTH_ROI_RADIUS + 1)

    roi = depth_raw[y1:y2, x1:x2]
    valid = roi[roi > 0]

    if valid.size == 0:
        return None

    z_raw = np.median(valid)
    return z_raw * depth_scale * 1000.0


def draw_coordinate_overlay(frame):
    for mm in [-200, -100, 100, 200]:
        x = int(PLATE_CENTER_X + mm / SCALE_X)
        y = int(PLATE_CENTER_Y - mm / SCALE_Y)

        cv2.line(frame, (x, PLATE_Y1), (x, PLATE_Y2), (80, 80, 80), 1)
        cv2.line(frame, (PLATE_X1, y), (PLATE_X2, y), (80, 80, 80), 1)

        cv2.putText(frame, f'{mm}', (x + 5, PLATE_CENTER_Y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180,180,180), 1)
        cv2.putText(frame, f'{mm}', (PLATE_CENTER_X + 8, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180,180,180), 1)

    cv2.rectangle(frame, (PLATE_X1, PLATE_Y1), (PLATE_X2, PLATE_Y2), (0, 0, 255), 2)
    cv2.line(frame, (PLATE_X1, PLATE_CENTER_Y), (PLATE_X2, PLATE_CENTER_Y), (0,255,0), 2)
    cv2.line(frame, (PLATE_CENTER_X, PLATE_Y1), (PLATE_CENTER_X, PLATE_Y2), (255,255,0), 2)

    cv2.putText(frame, 'X-', (PLATE_X1 + 10, PLATE_CENTER_Y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    cv2.putText(frame, 'X+', (PLATE_X2 - 35, PLATE_CENTER_Y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    cv2.putText(frame, 'Y+', (PLATE_CENTER_X + 8, PLATE_Y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
    cv2.putText(frame, 'Y-', (PLATE_CENTER_X + 8, PLATE_Y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)

    cv2.circle(frame, (PLATE_CENTER_X, PLATE_CENTER_Y), 6, (0, 0, 255), -1)
    cv2.putText(frame, '(0,0)', (PLATE_CENTER_X + 10, PLATE_CENTER_Y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

def draw_depth_legend(depth_img):
    h = depth_img.shape[0]
    x0 = depth_img.shape[1] - 50
    y0 = 20
    y1 = h - 20

    for y in range(y0, y1):
        ratio = (y - y0) / max((y1 - y0), 1)
        depth_mm = DEPTH_MIN_MM + ratio * (DEPTH_MAX_MM - DEPTH_MIN_MM)
        norm = int(255 * (depth_mm - DEPTH_MIN_MM) / (DEPTH_MAX_MM - DEPTH_MIN_MM))
        color = cv2.applyColorMap(np.uint8([[norm]]), cv2.COLORMAP_JET)[0][0]
        cv2.line(depth_img, (x0, y), (x0 + 30, y), tuple(int(c) for c in color), 1)


def contour_circularity(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return 0
    return 4 * math.pi * area / (perimeter ** 2)


def contour_centroid(contour):
    Mmom = cv2.moments(contour)
    if Mmom['m00'] == 0:
        return None
    return (int(Mmom['m10'] / Mmom['m00']), int(Mmom['m01'] / Mmom['m00']))


def detect_ball(gray, predicted=None):
    _, mask = cv2.threshold(gray, THRESHOLD, 255, cv2.THRESH_BINARY_INV)

    plate_mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)

    cv2.fillPoly(
        plate_mask,
        [rotated_quad.astype(np.int32)],
        255
    )

    kernel = np.ones(
        (2*MASK_MARGIN+1,
        2*MASK_MARGIN+1),
        np.uint8
    )

    plate_mask = cv2.erode(
        plate_mask,
        kernel,
        iterations=1
    )

    mask = cv2.bitwise_and(
    mask,
    plate_mask
    )
    
    kernel = np.ones((6, 6), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA or area > MAX_AREA:
            continue

        circ = contour_circularity(contour)
        if circ < MIN_CIRCULARITY:
            continue

        ctr = contour_centroid(contour)
        if ctr is None:
            continue

        if not (PLATE_X1 <= ctr[0] <= PLATE_X2 and PLATE_Y1 <= ctr[1] <= PLATE_Y2):
            continue

        candidates.append((contour, ctr, area))

    if not candidates:
        return None, mask

    if predicted is None:
        return max(candidates, key=lambda x: x[2]), mask

    valid = []
    for candidate in candidates:
        dist = math.hypot(candidate[1][0] - predicted[0], candidate[1][1] - predicted[1])
        if dist <= MAX_TRACK_DISTANCE:
            valid.append((candidate, dist))

    if valid:
        valid.sort(key=lambda x: x[1])
        return valid[0][0], mask

    return max(candidates, key=lambda x: x[2]), mask

# ================= REALSENSE =================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS_Depth)

profile = pipeline.start(config)
align = rs.align(rs.stream.color)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Intel RealSense SDK filters
# spatial = rs.spatial_filter()
# hole = rs.hole_filling_filter()
temporal = rs.temporal_filter()
temporal.set_option(
    rs.option.filter_smooth_alpha,
    TEMPORAL_ALPHA
)

temporal.set_option(
    rs.option.filter_smooth_delta,
    TEMPORAL_DELTA
)

prev_pos = None
velocity = None

# FPS measurement
fps = 0.0
fps_alpha = 0.1
prev_time = time.perf_counter()

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # RealSense filtering
        # depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        # depth_frame = hole.process(depth_frame)

        frame = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())

        frame = cv2.warpAffine(frame, ROT_MAT, (WIDTH, HEIGHT))
        frame = cv2.flip(frame, -1)
        depth_raw = cv2.warpAffine(depth_raw, ROT_MAT, (WIDTH, HEIGHT))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        predicted = None
        if prev_pos is not None and velocity is not None:
            predicted = (int(prev_pos[0] + velocity[0]), int(prev_pos[1] + velocity[1]))

        result, mask = detect_ball(gray, predicted)

        chosen = None
        ctrl_coords = None
        z_mm = None
        z_display = None
        z_mean = 0
        z_std = 0

        if result is not None:
            contour, chosen, area = result
            ctrl_coords = pixel_to_control_coords(chosen)

            z_mm = get_median_depth_mm(depth_raw, chosen[0], chosen[1], depth_scale)
            if z_mm is not None:

                z_plate = get_plate_reference_mm(plate_depth_reference, chosen[0], chosen[1])

                if z_plate > 0:
                    z_display = z_plate - z_mm
                else:
                    z_display = None

            if prev_pos is not None:
                velocity = (chosen[0] - prev_pos[0], chosen[1] - prev_pos[1])
            else:
                velocity = (0, 0)

            prev_pos = chosen
        else:
            prev_pos = None
            velocity = None

        display = frame.copy()
        draw_coordinate_overlay(display)

        if result is not None:
            cv2.drawContours(display, [contour], -1, (255, 0, 255), 2)
            cv2.circle(display, chosen, 5, (0, 0, 255), -1)

        if ctrl_coords is not None and z_display is not None:
            last_valid_ctrl_coords = ctrl_coords
            last_valid_z = z_display          
            send_ball_position_xyz_mm(ctrl_coords, z_display,detected_flag=1)

            cv2.putText(display, f'X: {ctrl_coords[0]:+6.1f} mm', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
            cv2.putText(display, f'Y: {ctrl_coords[1]:+6.1f} mm', (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)

            if z_display is not None:
                z_history.append(z_display)
                if len(z_history) >= 30:
                    z_mean = np.mean(z_history)
                    z_std = np.std(z_history)
                z_label = f'Z: {z_display:+6.1f} mm'
                cv2.putText(display, z_label, (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        else:
            send_ball_position_xyz_mm(last_valid_ctrl_coords, last_valid_z, detected_flag=0)

        # ================= FPS DISPLAY =================
        fps_text = f'FPS: {fps:5.1f}'

        (text_w, text_h), _ = cv2.getTextSize(
            fps_text,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            2
        )

        cv2.putText(
            display,
            fps_text,
            (WIDTH - text_w - 15, HEIGHT - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        if DEPTH_CALIBRATION:
            cv2.putText(
                display,
                f'Z mean: {z_mean:7.2f} mm',
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255,255,255),
                2
            )

            cv2.putText(
                display,
                f'Z std : {z_std:6.2f} mm',
                (20, 150),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255,255,255),
                2
            )
        cv2.imshow('Phase E3 Depth Measurement Improvement', display)

        if SHOW_DEPTH_VIEW:
            depth_mm = depth_raw.astype(np.float32) * depth_scale * 1000.0
            depth_clipped = np.clip(depth_mm, DEPTH_MIN_MM, DEPTH_MAX_MM)
            depth_norm = ((depth_clipped - DEPTH_MIN_MM) / (DEPTH_MAX_MM - DEPTH_MIN_MM) * 255).astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            draw_depth_legend(depth_vis)
            if chosen is not None:
                cv2.circle(depth_vis, chosen, 5, (255,255,255), -1)
            cv2.imshow('Depth View', depth_vis)

        if SHOW_MASK:
            cv2.imshow('Binary Mask', mask)

        # ================= FPS UPDATE =================
        current_time = time.perf_counter()
        dt = current_time - prev_time

        if dt > 0:
            fps_new = 1.0 / dt
            fps = (1.0 - fps_alpha) * fps + fps_alpha * fps_new

        prev_time = current_time

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()