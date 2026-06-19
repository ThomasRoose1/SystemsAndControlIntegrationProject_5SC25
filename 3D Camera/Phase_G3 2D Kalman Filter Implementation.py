import socket
import struct
import cv2
import numpy as np
import pyrealsense2 as rs
import math
import json
import time
from collections import deque

# ================= CONFIG =================
# Pipeline parameters
WIDTH = 640
HEIGHT = 480
FPS = 60
FPS_Depth = 90

#Calibration parameters
CALIBRATION_FILE = 'plate_calibration.json'
PLATE_SIZE_MM = 400.0
GRID_SPACING_MM = 100.0

# Mask parameters
MAX_TRACK_DISTANCE = 120
MASK_MARGIN = 15 # Margin to erode plate mask to avoid edge artifacts
THRESHOLD = 100 # Adjust based on lighting conditions and ball color
MIN_CIRCULARITY = 0.75 # 1.0 is a perfect circle, lower values allow more distortion
MIN_AREA = 200 
MAX_AREA = 2000 # Adjust based on expected ball size in pixels
# Potential continuous auto-calibration --> MIN_RADIUS = ball_radius_ref - 3*radius_std // MAX_RADIUS = ball_radius_ref + 3*radius_std
MIN_RADIUS = 15 # Need to be calibrated
MAX_RADIUS = 30 # Need to be calibrated
EDGE_MARGIN = 30 # Need to be calibrated
USE_EDGE_COMPENSATION = True # If True, allows detection of partially visible balls near plate edges by compensating with expected radius

BALL_RADIUS_ALPHA = 0.2 # 1.0 = no smoothing, 0.1 = max smoothing (very slow response)
XY_FILTER_ALPHA = 0.6 # 1.0 = no filtering, 0.1 = max filtering 

# Depth Parameters
DEPTH_MIN_MM = 400
DEPTH_MAX_MM = 900
XY_FILTER_ALPHA_MIN = 0.3
XY_FILTER_ALPHA_MAX = 0.7
ADAPTIVE_FILTER_SPEED = 50.0
TEMPORAL_ALPHA = 0.4 # 0.0 = no temporal smoothing, 1.0 = max smoothing (very slow response)
TEMPORAL_DELTA = 20 # 0 = no temporal threshold, higher values reject more sudden changes (noise) but can cause lag
DEPTH_ROI_RADIUS = 5 # Radius for median depth calculation (11x11 window)

# UDP
serverAddressPort = ("192.168.140.8", 49001)
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Debug print
DEBUG_PRINT_UDP = False
SHOW_MASK = True
SHOW_DEPTH_VIEW = False
SHOW_XY_STATS = False
SHOW_Z_STATS = False
SHOW_RADIUS_CALIBRATION = False
SHOW_EDGE_ZONE = False
SHOW_TRACKING_DEBUG = False

# Other parameters
STATS_WINDOW = 300
x_history = deque(maxlen=STATS_WINDOW)
y_history = deque(maxlen=STATS_WINDOW)
z_history = deque(maxlen=STATS_WINDOW)
radius_history = deque(maxlen=STATS_WINDOW)


# ================= LOAD CALIBRATION =================
with open(CALIBRATION_FILE, 'r') as f:
    calib = json.load(f)

plate_quad = np.array(calib['plate_quad'], dtype=np.float32)
TL, TR, BR, BL = plate_quad

# Horizontal edges
top_angle = math.degrees(math.atan2(TR[1] - TL[1], TR[0] - TL[0]))
bottom_angle = math.degrees(math.atan2(BR[1] - BL[1], BR[0] - BL[0]))

# Vertical edges
left_angle = math.degrees(math.atan2(BL[1] - TL[1], BL[0] - TL[0])) - 90.0
right_angle = math.degrees(math.atan2(BR[1] - TR[1], BR[0] - TR[0])) - 90.0

angle_deg = np.mean([ top_angle, bottom_angle, left_angle, right_angle])

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
        detected_flag])
    if DEBUG_PRINT_UDP:
        print(f"UDP -> X={x_mm}, Y={y_mm}, Z={z_mm}, Flag={detected_flag}")
    UDPClientSocket.sendto(packet, serverAddressPort)


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

        cv2.putText(frame, f'{mm}', (x + 5, PLATE_CENTER_Y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180,180,180), 1)
        cv2.putText(frame, f'{mm}', (PLATE_CENTER_X + 8, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180,180,180), 1)

    cv2.rectangle(frame, (PLATE_X1, PLATE_Y1), (PLATE_X2, PLATE_Y2), (0, 0, 255), 2)
    cv2.line(frame, (PLATE_X1, PLATE_CENTER_Y), (PLATE_X2, PLATE_CENTER_Y), (0,255,0), 2)
    cv2.line(frame, (PLATE_CENTER_X, PLATE_Y1), (PLATE_CENTER_X, PLATE_Y2), (255,255,0), 2)

    cv2.putText(frame, 'X-', (PLATE_X1 + 10, PLATE_CENTER_Y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    cv2.putText(frame, 'X+', (PLATE_X2 - 35, PLATE_CENTER_Y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    cv2.putText(frame, 'Y+', (PLATE_CENTER_X + 8, PLATE_Y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
    cv2.putText(frame, 'Y-', (PLATE_CENTER_X + 8, PLATE_Y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)

    cv2.circle(frame, (PLATE_CENTER_X, PLATE_CENTER_Y), 6, (0, 0, 255), -1)
    cv2.putText(frame, '(0,0)', (PLATE_CENTER_X + 10, PLATE_CENTER_Y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

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

def contour_circle_fit(contour):
    (x, y), radius = cv2.minEnclosingCircle(contour)
    return ((x, y),radius)

def edge_compensated_center(center, radius):
    x, y = center
    if ball_radius_ref is None:
        return center
    
    r = ball_radius_ref

    if x < PLATE_X1 + EDGE_MARGIN:
        x = max(x, PLATE_X1 + r)
    elif x > PLATE_X2 - EDGE_MARGIN:
        x = min(x, PLATE_X2 - r)

    if y < PLATE_Y1 + EDGE_MARGIN:
        y = max(y, PLATE_Y1 + r)
    elif y > PLATE_Y2 - EDGE_MARGIN:
        y = min(y, PLATE_Y2 - r)

    return (x, y)

def compute_adaptive_alpha(speed_px):

    alpha = np.interp(
        speed_px,
        [0, ADAPTIVE_FILTER_SPEED],
        [XY_FILTER_ALPHA_MIN,
         XY_FILTER_ALPHA_MAX]
    )

    return float(alpha)

def detect_ball(gray, predicted=None):
    gray = cv2.GaussianBlur(gray,(5,5),0) # gaussian Blur to reduce noise and improve thresholding
    _, mask = cv2.threshold(gray,THRESHOLD,255,cv2.THRESH_BINARY_INV) # binary inverse thresholding to get white ball on black background
    plate_mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8) # create empty mask for plate region

    cv2.fillPoly(plate_mask,[rotated_quad.astype(np.int32)],255)

    kernel = np.ones((2*MASK_MARGIN+1,2*MASK_MARGIN+1),np.uint8)
    plate_mask = cv2.erode(plate_mask,kernel,iterations=1)
    mask = cv2.bitwise_and(mask,plate_mask)
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    filter_vis = cv2.cvtColor(mask.copy(), cv2.COLOR_GRAY2BGR)
    
    candidates = []

    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        contour_near_edge = (
            x < PLATE_X1 + EDGE_MARGIN or
            x + w > PLATE_X2 - EDGE_MARGIN or
            y < PLATE_Y1 + EDGE_MARGIN or
            y + h > PLATE_Y2 - EDGE_MARGIN)
        
        if area < MIN_AREA or area > MAX_AREA:
            cv2.drawContours(filter_vis, [contour], -1, (0,0,255), 2)
            continue

        circ = contour_circularity(contour)
        if (not contour_near_edge) and circ < MIN_CIRCULARITY:
            cv2.drawContours(filter_vis, [contour], -1, (0,165,255), 2)
            continue

        ctr, radius = contour_circle_fit(contour)
        # print(f"Radius={radius:.2f}")
        contour_near_edge = (ctr[0] < PLATE_X1 + EDGE_MARGIN or
                             ctr[0] > PLATE_X2 - EDGE_MARGIN or
                             ctr[1] < PLATE_Y1 + EDGE_MARGIN or
                             ctr[1] > PLATE_Y2 - EDGE_MARGIN)
        
        if radius < MIN_RADIUS:
            cv2.drawContours(filter_vis, [contour], -1, (255,0,0), 2)
            continue

        if radius > MAX_RADIUS:
            cv2.drawContours(filter_vis, [contour], -1, (255,0,0), 2)
            continue

        if not (PLATE_X1 <= ctr[0] <= PLATE_X2 and PLATE_Y1 <= ctr[1] <= PLATE_Y2):
            continue
        
        cv2.drawContours(filter_vis, [contour], -1, (0,255,0), 2)
        candidates.append((contour,ctr,area,radius))

    if not candidates:
        return None, mask, filter_vis

    if predicted is None:
        return max(candidates, key=lambda x: x[2]), mask, filter_vis

    valid = []
    for candidate in candidates:
        dist = math.hypot(candidate[1][0] - predicted[0], candidate[1][1] - predicted[1])
        if dist <= MAX_TRACK_DISTANCE:
            valid.append((candidate, dist))

    if valid:
        valid.sort(key=lambda x: x[1])
        return valid[0][0], mask, filter_vis

    return max(candidates, key=lambda x: x[2]), mask, filter_vis

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
temporal = rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha, TEMPORAL_ALPHA)
temporal.set_option(rs.option.filter_smooth_delta,TEMPORAL_DELTA)

# Values initialization
prev_pos = None
velocity = None
lost_frames = 0
LOST_TIMEOUT_FRAMES = 5
filtered_pos = None
ball_radius_ref = None
z_reference_mm = None
last_valid_ctrl_coords = (0.0, 0.0)
last_valid_z = 0.0

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
        depth_frame = temporal.process(depth_frame)

        frame = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())

        frame = cv2.warpAffine(frame, ROT_MAT, (WIDTH, HEIGHT))
        frame = cv2.flip(frame, -1)
        depth_raw = cv2.warpAffine(depth_raw, ROT_MAT, (WIDTH, HEIGHT))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        predicted = None
        if prev_pos is not None and velocity is not None:
            predicted = (int(prev_pos[0] + velocity[0]), int(prev_pos[1] + velocity[1]))
            predicted_pos = predicted

        result, mask, filter_vis = detect_ball(gray, predicted)

        chosen = None
        ctrl_coords = None
        z_mm = None
        z_display = None
        raw_center = None
        comp_center = None
        near_edge = False

        # Stats initialization
        x_mean = 0
        x_std = 0
        y_mean = 0
        y_std = 0
        z_mean = 0
        z_std = 0
        radius_mean = 0
        radius_std = 0

        vx_mm = 0
        vy_mm = 0
        speed_mm = 0

        if result is not None:
            contour, chosen, area, radius = result
            lost_frames = 0
            raw_center = chosen

            radius_history.append(radius)
            if len(radius_history) >= 30:
                radius_mean = np.mean(radius_history)
                radius_std = np.std(radius_history)

                radius_min = np.min(radius_history)
                radius_max = np.max(radius_history)

            near_edge = (chosen[0] < PLATE_X1 + EDGE_MARGIN or
                         chosen[0] > PLATE_X2 - EDGE_MARGIN or
                         chosen[1] < PLATE_Y1 + EDGE_MARGIN or
                         chosen[1] > PLATE_Y2 - EDGE_MARGIN)
            
            if not near_edge:
                if ball_radius_ref is None:
                    ball_radius_ref = radius
                else:
                    ball_radius_ref = ((1.0 - BALL_RADIUS_ALPHA) * ball_radius_ref + BALL_RADIUS_ALPHA * radius)

            if USE_EDGE_COMPENSATION:
                comp_center = edge_compensated_center(chosen, radius)
                chosen = comp_center
            else:
                comp_center = chosen

            if velocity is None:
                alpha_xy = XY_FILTER_ALPHA_MIN
            else:
                speed_px = math.hypot(velocity[0], velocity[1])
                alpha_xy = compute_adaptive_alpha(speed_px)
                # print(f"Adaptive filter -> {alpha_xy:.3f} (speed={speed_px:.2f}px)")

            if filtered_pos is None:
                filtered_pos = chosen
            else:
                filtered_pos = (alpha_xy * chosen[0] + (1.0 - alpha_xy) * filtered_pos[0], 
                                alpha_xy * chosen[1] + (1.0 - alpha_xy) * filtered_pos[1])
            chosen = filtered_pos
            ctrl_coords = pixel_to_control_coords(chosen)

            z_mm = get_median_depth_mm(depth_raw, int(round(chosen[0])), int(round(chosen[1])), depth_scale)

            if z_mm is not None:
                z_display = z_mm if z_reference_mm is None else z_reference_mm - z_mm

            if z_display is not None:
                last_valid_ctrl_coords = ctrl_coords
                last_valid_z = z_display
                
            if prev_pos is not None:
                velocity = (chosen[0] - prev_pos[0], chosen[1] - prev_pos[1])
                vx_mm = velocity[0] * SCALE_X * fps
                vy_mm = -velocity[1] * SCALE_Y * fps
                speed_mm = math.hypot(vx_mm, vy_mm)
            else:
                velocity = (0, 0)

            prev_pos = chosen
        else:
            lost_frames += 1
            if lost_frames <= LOST_TIMEOUT_FRAMES:
                if (prev_pos is not None and velocity is not None):
                    prev_pos = (prev_pos[0] + velocity[0], prev_pos[1] + velocity[1])
            else:
                prev_pos = None
                velocity = None
                filtered_pos = None

        display = frame.copy()
        draw_coordinate_overlay(display)

        if result is not None:
            cv2.drawContours(display, [contour], -1, (255, 0, 255), 2)
            
            raw_int = (int(round(raw_center[0])), int(round(raw_center[1])))
            comp_int = (int(round(comp_center[0])), int(round(comp_center[1])))

            filt_int = (int(round(chosen[0])), int(round(chosen[1])))
            # Fitted circle
            cv2.circle(display, raw_int, int(round(radius)), (0,255,255), 2)

            if SHOW_TRACKING_DEBUG:
                # Raw circle fit center
                cv2.circle(display, raw_int, 4, (0,255,0), -1)

                # Edge compensated center
                cv2.circle(display, comp_int, 4, (0,255,255), -1)

                # Final EMA-filtered center
                cv2.circle(display, filt_int, 6, (0,0,255), -1)
            else:
                cv2.circle(display, filt_int, 6, (0,0,255), -1)

        if ctrl_coords is not None and z_display is not None:
            x_history.append(ctrl_coords[0])
            y_history.append(ctrl_coords[1])
            if len(x_history) >= 30:

                x_mean = np.mean(x_history)
                x_std = np.std(x_history)

                y_mean = np.mean(y_history)
                y_std = np.std(y_history)

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
        (text_w, text_h), _ = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        cv2.putText(display, fps_text, (WIDTH - text_w - 15, HEIGHT - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # ================= DEBUG OVERLAYS =================

        if SHOW_XY_STATS:
            cv2.putText(display, f'X mean: {x_mean:7.2f} mm', (20,120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(display, f'X std : {x_std:6.2f} mm', (20,150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(display, f'Y mean: {y_mean:7.2f} mm', (20,180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(display, f'Y std : {y_std:6.2f} mm', (20,210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        
        if SHOW_Z_STATS:
            cv2.putText(display, f'Z mean: {z_mean:7.2f} mm', (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(display, f'Z std : {z_std:6.2f} mm', (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        
        if SHOW_RADIUS_CALIBRATION and len(radius_history) >= 30:
            # print(f'Radius calibration ON')
            cv2.putText(display, f'R mean: {radius_mean:.2f}px', (20, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(display, f'R std : {radius_std:.2f}px', (20, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(display, f'R min : {radius_min:.2f}px', (20, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(display, f'R max : {radius_max:.2f}px', (20, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        
        if SHOW_TRACKING_DEBUG:
            cv2.putText(display, 'GREEN  = Raw Fit', (20,300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            cv2.putText(display, 'YELLOW = Edge Comp', (20,325), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
            cv2.putText(display, 'RED    = Filtered', (20,350), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
            cv2.putText(display, f'Vx: {vx_mm:+6.1f} mm/s', (20, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
            cv2.putText(display, f'Vy: {vy_mm:+6.1f} mm/s', (20, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
            cv2.putText(display, f'|V|: {speed_mm:6.1f} mm/s', (20, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

            if predicted is not None:
                cv2.circle(display, predicted, 6, (255,0,0), 2)
        
        if near_edge:
            # print(f'Near edge ON')
            cv2.putText(display, 'EDGE MODE', (20,450), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        
        if SHOW_EDGE_ZONE:
            cv2.rectangle(display, (PLATE_X1 + EDGE_MARGIN, PLATE_Y1 + EDGE_MARGIN), (PLATE_X2 - EDGE_MARGIN, PLATE_Y2 - EDGE_MARGIN), (100,100,255), 1)

        # if ball_radius_ref is not None:
        #     cv2.putText(display, f'Rref: {ball_radius_ref:.1f}px', (20,300), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)       

        if SHOW_DEPTH_VIEW:
            depth_mm = depth_raw.astype(np.float32) * depth_scale * 1000.0
            depth_center = (int(round(chosen[0])), int(round(chosen[1])))
            depth_clipped = np.clip(depth_mm, DEPTH_MIN_MM, DEPTH_MAX_MM)
            depth_norm = ((depth_clipped - DEPTH_MIN_MM) / (DEPTH_MAX_MM - DEPTH_MIN_MM) * 255).astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            draw_depth_legend(depth_vis)
            if chosen is not None:
                cv2.circle(depth_vis, depth_center, 5, (255,255,255), -1)
            cv2.imshow('Depth View', depth_vis)

        if SHOW_MASK:
            cv2.putText(filter_vis, 'GREEN = Accepted', (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(filter_vis, 'RED = Area Reject', (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            cv2.putText(filter_vis, 'ORANGE = Circularity Reject', (10,75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
            cv2.putText(filter_vis, 'BLUE = Radius Reject', (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
            cv2.imshow('Final Filter Visualization', filter_vis)

        cv2.imshow('Phase_G1 2D Tracking Improvement', display)
        # ================= FPS UPDATE =================
        current_time = time.perf_counter()
        dt = current_time - prev_time

        if dt > 0:
            fps_new = 1.0 / dt
            fps = (1.0 - fps_alpha) * fps + fps_alpha * fps_new

        prev_time = current_time

        key = cv2.waitKey(1) & 0xFF
        if key == ord('z') and z_mm is not None:
            z_reference_mm = z_mm
            print(f'Z reference set to {z_reference_mm:.1f} mm')
        if key == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()