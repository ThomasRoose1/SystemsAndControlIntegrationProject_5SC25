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
PLATE_SIZE_MM = 390.0
GRID_SPACING_MM = 100.0

# Mask parameters
MAX_TRACK_DISTANCE = 120
MASK_MARGIN = 15 # Margin to erode plate mask to avoid edge artifacts
THRESHOLD = 95 # Adjust based on lighting conditions and ball color
MIN_CIRCULARITY = 0.75 # 1.0 is a perfect circle, lower values allow more distortion
MIN_AREA = 200 
MAX_AREA = 2000 # Adjust based on expected ball size in pixels
# Potential continuous auto-calibration --> MIN_RADIUS = ball_radius_ref - 3*radius_std // MAX_RADIUS = ball_radius_ref + 3*radius_std
MIN_RADIUS = 15 # Need to be calibrated
MAX_RADIUS = 30 # Need to be calibrated
EDGE_MARGIN = 40 # Need to be calibrated
USE_EDGE_COMPENSATION = True # If True, allows detection of partially visible balls near plate edges by compensating with expected radius
USE_GAUSSIAN_BLUR = False
USE_RADIUS_FILTER = False
USE_HOMOGRAPHY = True

BALL_RADIUS_ALPHA = 0.2 # 1.0 = no smoothing, 0.1 = max smoothing (very slow response)
XY_FILTER_ALPHA = 0.6 # 1.0 = no filtering, 0.1 = max filtering 

# Depth Parameters
DEPTH_MIN_MM = 400
DEPTH_MAX_MM = 900
XY_FILTER_ALPHA_MIN = 0.3
XY_FILTER_ALPHA_MAX = 0.7
ADAPTIVE_FILTER_SPEED = 50.0
TEMPORAL_ALPHA = 0.6 # 0.0 = no temporal smoothing, 1.0 = max smoothing (very slow response)
TEMPORAL_DELTA = 20 # 0 = no temporal threshold, higher values reject more sudden changes (noise) but can cause lag
DEPTH_ROI_RADIUS = 5 # Radius for median depth calculation (11x11 window)
# Robust depth estimation
USE_ROBUST_DEPTH = False
DEPTH_OUTLIER_SIGMA = 2.0
USE_Z_FILTER = True
USE_ADAPTIVE_Z_FILTER = True

# Z EMA parameters
Z_FILTER_ALPHA = 0.25          # Used when adaptive filtering is disabled

Z_FILTER_ALPHA_MIN = 0.2      # Strong filtering
Z_FILTER_ALPHA_MAX = 0.65      # Weak filtering

ADAPTIVE_Z_SPEED = 250.0        # mm/s
USE_SPATIAL_FILTER = True
SPATIAL_MAGNITUDE = 2
SPATIAL_ALPHA = 0.5
SPATIAL_DELTA = 20

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
SHOW_DEPTH_ROI = False
PROFILE = True
timing_stats = {
    "acquisition": [],
    "detection": [],
    "tracking": [],
    "depth": [],
    "udp": [],
    "display": [],
    "total": []
}

# Other parameters
STATS_WINDOW = 300
x_history = deque(maxlen=STATS_WINDOW)
y_history = deque(maxlen=STATS_WINDOW)
z_history = deque(maxlen=STATS_WINDOW)
radius_history = deque(maxlen=STATS_WINDOW)

# Reference parameters
REFERENCE_MODE = "" # "LIVE" = use current mouse position as reference, "PATH" = draw reference path by dragging mouse, "" = no reference
mouse_down = False


# ================= LOAD CALIBRATION =================
with open(CALIBRATION_FILE, 'r') as f:
    calib = json.load(f)

plate_quad = np.array(calib['plate_quad'], dtype=np.float32)
TL, TR, BR, BL = plate_quad

# ================= HOMOGRAPHY =================
image_points = np.array([TL, TR, BR, BL], dtype=np.float32)
half = PLATE_SIZE_MM/2
world_points = np.array([[-half,-half], [half,-half], [half, half], [-half, half]], dtype=np.float32)

H, _ = cv2.findHomography(image_points, world_points)

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

    # -------------------------
    # Original linear mapping
    # -------------------------
    if not USE_HOMOGRAPHY:
        x_mm = (pixel_xy[0] - PLATE_CENTER_X) * SCALE_X
        y_mm = (PLATE_CENTER_Y - pixel_xy[1]) * SCALE_Y

        return (x_mm, y_mm)

    # -------------------------
    # Homography mapping
    # -------------------------
    pt = np.array([[[pixel_xy[0], pixel_xy[1]]]], dtype=np.float32)
    world = cv2.perspectiveTransform(pt,H)

    return (float(world[0,0,0]), float(-world[0,0,1]))

def reference_pixel_to_mm(pixel_xy):
    return pixel_to_control_coords(pixel_xy)

def mouse_callback(event, x, y, flags, param):
    global ref_pixel
    global ref_path_pixels
    global mouse_down

    if REFERENCE_MODE == "LIVE":
        if event == cv2.EVENT_MOUSEMOVE:
            ref_pixel = (x, y)

    elif REFERENCE_MODE == "PATH":
        if event == cv2.EVENT_LBUTTONDOWN:
            mouse_down = True
            ref_path_pixels = [(x, y)]

        elif event == cv2.EVENT_MOUSEMOVE and mouse_down:
            ref_path_pixels.append((x, y))

        elif event == cv2.EVENT_LBUTTONUP:
            mouse_down = False

def send_ball_position_xyz_mm(ctrl_xy, z_value, detected_flag):

    x_mm = round(float(ctrl_xy[0]),2)
    y_mm = round(float(ctrl_xy[1]),2)
    z_mm = round(float(z_value),2)

    packet = struct.pack(
        '>fffb',
        x_mm,
        y_mm,
        z_mm,
        bool(detected_flag)
    )
    if DEBUG_PRINT_UDP:
        print(f"UDP -> X={x_mm}, Y={y_mm}, Z={z_mm}, Flag={detected_flag}")
    UDPClientSocket.sendto(packet, serverAddressPort)

def send_reference_xy(ref_xy, ref_flag):

    x_ref = round(float(ref_xy[0]),2)
    y_ref = round(float(ref_xy[1]),2)

    packet = struct.pack(
        '>ffb',
        x_ref,
        y_ref,
        bool(ref_flag)
    )
    if DEBUG_PRINT_UDP:
        print(f"UDP -> X_ref{x_ref}, Y_ref={y_ref}, Flag={ref_flag}")
    UDPClientSocket.sendto(packet, ("192.168.140.8",49002))


def get_mean_depth_mm(depth_raw, x, y, depth_scale):
    depth_radius = max(3, int(ball_radius_ref * 0.4)) if ball_radius_ref is not None else DEPTH_ROI_RADIUS
    
    global current_depth_radius
    global current_depth_mask
    global current_depth_x1
    global current_depth_y1
    current_depth_radius = depth_radius

    x1 = max(0, x - depth_radius)
    x2 = min(depth_raw.shape[1], x + depth_radius + 1)

    y1 = max(0, y - depth_radius)
    y2 = min(depth_raw.shape[0], y + depth_radius + 1)

    roi = depth_raw[y1:y2, x1:x2]
    yy, xx = np.ogrid[:roi.shape[0], :roi.shape[1]]

    cx = x - x1
    cy = y - y1

    circle_mask = ((xx - cx)**2 + (yy - cy)**2) <= depth_radius**2

    accepted_mask = np.logical_and(circle_mask, roi > 0)
    current_depth_mask = accepted_mask
    current_depth_x1 = x1
    current_depth_y1 = y1

    valid = roi[accepted_mask]
    valid = valid.astype(np.float32)
    if valid.size == 0:
        return None

    if USE_ROBUST_DEPTH:
        median = np.median(valid)

        sigma = np.std(valid)
        if sigma > 0:
            valid = valid[np.abs(valid - median) < DEPTH_OUTLIER_SIGMA * sigma]
        z_raw = np.mean(valid)

    else:
        z_raw = np.mean(valid)

    return float(z_raw * depth_scale * 1000.0)


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

def compute_adaptive_z_alpha(speed_mm):

    alpha = np.interp(
        speed_mm,
        [0.0, ADAPTIVE_Z_SPEED],
        [Z_FILTER_ALPHA_MIN,
         Z_FILTER_ALPHA_MAX]
    )

    return float(alpha)

def detect_ball(gray, predicted=None):
    if USE_GAUSSIAN_BLUR:
        gray = cv2.GaussianBlur(gray, (5,5), 0)
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
        
        if USE_RADIUS_FILTER:

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

spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, SPATIAL_MAGNITUDE)
spatial.set_option( rs.option.filter_smooth_alpha, SPATIAL_ALPHA)
spatial.set_option( rs.option.filter_smooth_delta, SPATIAL_DELTA)

# Values initialization
prev_pos = None
velocity = None
lost_frames = 0
LOST_TIMEOUT_FRAMES = 5
filtered_pos = None
filtered_z = None
ball_radius_ref = None
z_reference_mm = None
last_valid_ctrl_coords = (0.0, 0.0)
last_valid_z = 0.0
current_depth_radius = DEPTH_ROI_RADIUS
current_depth_mask = None
current_depth_x1 = 0
current_depth_y1 = 0

ref_pixel = (PLATE_CENTER_X, PLATE_CENTER_Y)
ref_path_pixels = []

playback_index = 0
playback_active = False

# FPS measurement
fps_counter = 0
fps_timer = time.perf_counter()
camera_fps = 0
camera_fps_timestamp = 0
prev_timestamp = None

WINDOW_NAME = 'Final Code'
cv2.namedWindow(WINDOW_NAME)
cv2.setMouseCallback(WINDOW_NAME, mouse_callback)

try:
    while True:
        total_start = time.perf_counter()
        t0 = time.perf_counter()

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        timestamp = color_frame.get_timestamp()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        if USE_SPATIAL_FILTER:

            depth_frame = spatial.process(depth_frame)

        depth_frame = temporal.process(depth_frame)

        frame = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())

        t1 = time.perf_counter()

        frame = cv2.warpAffine(frame, ROT_MAT, (WIDTH, HEIGHT))
        frame = cv2.flip(frame, -1)
        depth_raw = cv2.warpAffine(depth_raw, ROT_MAT, (WIDTH, HEIGHT))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        predicted = None
        if prev_pos is not None and velocity is not None:
            predicted = (int(prev_pos[0] + velocity[0]), int(prev_pos[1] + velocity[1]))
            predicted_pos = predicted

        t_det_start = time.perf_counter()
        result, mask, filter_vis = detect_ball(gray, predicted)
        t2 = time.perf_counter()

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

        ref_mm = (0.0, 0.0)

        t_track_start = time.perf_counter()
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

            # Compute velocity before Z filtering
            if prev_pos is not None:

                velocity = (
                    chosen[0] - prev_pos[0],
                    chosen[1] - prev_pos[1]
                )

                vx_mm = (
                    ctrl_coords[0]
                    - last_valid_ctrl_coords[0]
                ) * camera_fps

                vy_mm = (
                    ctrl_coords[1]
                    - last_valid_ctrl_coords[1]
                ) * camera_fps

                speed_mm = math.hypot(
                    vx_mm,
                    vy_mm
                )

            else:

                velocity = (0,0)
                speed_mm = 0.0

            t_depth_start = time.perf_counter()
            z_mm = get_mean_depth_mm(depth_raw, int(round(chosen[0])), int(round(chosen[1])), depth_scale)
            t4 = time.perf_counter()

            if z_mm is not None:
                if USE_Z_FILTER:
                    if USE_ADAPTIVE_Z_FILTER:
                        alpha_z = compute_adaptive_z_alpha(speed_mm)

                    else:
                        alpha_z = Z_FILTER_ALPHA

                    if filtered_z is None:
                        filtered_z = z_mm

                    else:
                        filtered_z = (alpha_z * z_mm + (1.0 - alpha_z) * filtered_z)

                    z_mm = filtered_z

                if z_reference_mm is None:
                    z_display = z_mm

                else:
                    z_display = z_reference_mm - z_mm

            if z_display is not None:
                last_valid_ctrl_coords = ctrl_coords
                last_valid_z = z_display

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
                filtered_z = None
        t3 = time.perf_counter()

        display = frame.copy()
        draw_coordinate_overlay(display)

        if REFERENCE_MODE != "":
            cv2.circle(display, (int(ref_pixel[0]), int(ref_pixel[1])), 8, (255,255,255), -1)
        
        if (REFERENCE_MODE == "PATH" and len(ref_path_pixels) > 1):
            pts = np.array(ref_path_pixels, dtype=np.int32)
            cv2.polylines(display, [pts], False, (255,255,255), 2)

        if result is not None:

            raw_int = (int(round(raw_center[0])), int(round(raw_center[1])))
            comp_int = (int(round(comp_center[0])), int(round(comp_center[1])))
            filt_int = (int(round(chosen[0])), int(round(chosen[1])))

            if SHOW_DEPTH_ROI:
                cv2.circle(display, filt_int, current_depth_radius, (255,255,255), 1)

                if current_depth_mask is not None:
                    ys, xs = np.where(current_depth_mask)

                    for px, py in zip(xs, ys):
                        cv2.circle(display, (current_depth_x1 + px, current_depth_y1 + py), 1, (0,255,0), -1)

            if (not near_edge) or (not USE_EDGE_COMPENSATION):
                cv2.drawContours(display, [contour], -1, (255,0,255), 2)
                cv2.circle(display, filt_int, 6, (0,0,255), -1)

            else:
                cv2.circle(display, raw_int, int(round(radius)), (0,255,255), 2)
                cv2.circle(display, filt_int, 6, (0,0,255), -1)

            if SHOW_TRACKING_DEBUG:
                # Raw circle fit center
                cv2.circle(display, raw_int, 4, (0,255,0), -1)

                # Edge compensated center
                cv2.circle(display, comp_int, 4, (0,255,255), -1)

                # Final EMA-filtered center
                cv2.circle(display, filt_int, 6, (0,0,255), -1)
            else:
                cv2.circle(display, filt_int, 6, (0,0,255), -1)

        t_udp_start = time.perf_counter() 
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
        t5 = time.perf_counter()
        
        # ================= FPS UPDATE =================
        fps_counter += 1
        if prev_timestamp is not None:
            dt_camera = timestamp - prev_timestamp
            if dt_camera > 0:
                camera_fps_timestamp = 1000.0 / dt_camera
        prev_timestamp = timestamp

        elapsed = time.perf_counter() - fps_timer
        if elapsed >= 1.0:
            camera_fps = fps_counter / elapsed

            fps_counter = 0
            fps_timer = time.perf_counter()

        # ================= FPS DISPLAY =================
        cv2.putText(display, f'Loop FPS: {camera_fps:5.1f}', (WIDTH - 180, HEIGHT - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.putText(display, f'Cam FPS: {camera_fps_timestamp:5.1f}', (WIDTH - 180, HEIGHT - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

        # ================= DEBUG OVERLAYS =================

        if SHOW_XY_STATS:
            cv2.putText(display, f'X mean: {x_mean:7.2f} mm', (20,120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(display, f'X std : {x_std:6.2f} mm', (20,150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(display, f'Y mean: {y_mean:7.2f} mm', (20,180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(display, f'Y std : {y_std:6.2f} mm', (20,210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        
        if SHOW_Z_STATS:
            cv2.putText(display, f'Z mean: {z_mean:7.2f} mm', (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(display, f'Z std : {z_std:6.2f} mm', (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            
            depth_pixel_count = 0

            if current_depth_mask is not None:
                depth_pixel_count = np.count_nonzero(current_depth_mask)
            cv2.putText(display, f'Depth pixels: {depth_pixel_count}', (20,180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        
        if SHOW_RADIUS_CALIBRATION and len(radius_history) >= 30:
            # print(f'Radius calibration ON')
            cv2.putText(display, f'R mean: {radius_mean:.2f}px', (20, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(display, f'R std : {radius_std:.2f}px', (20, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(display, f'R min : {radius_min:.2f}px', (20, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(display, f'R max : {radius_max:.2f}px', (20, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            if ball_radius_ref is not None:
                cv2.putText(display, f'Rref: {ball_radius_ref:.1f}px', (20,300), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        
        if SHOW_TRACKING_DEBUG:
            cv2.putText(display, 'GREEN  = Raw Fit', (20,300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            cv2.putText(display, 'YELLOW = Edge Comp', (20,325), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
            cv2.putText(display, 'RED    = Filtered', (20,350), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
            cv2.putText(display, f'Vx: {vx_mm:+6.1f} mm/s', (20, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
            cv2.putText(display, f'Vy: {vy_mm:+6.1f} mm/s', (20, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
            cv2.putText(display, f'|V|: {speed_mm:6.1f} mm/s', (20, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

            if predicted is not None:
                cv2.circle(display, predicted, 6, (255,0,0), 2)
        
        if SHOW_EDGE_ZONE:
            cv2.rectangle(display, (PLATE_X1 + EDGE_MARGIN, PLATE_Y1 + EDGE_MARGIN), (PLATE_X2 - EDGE_MARGIN, PLATE_Y2 - EDGE_MARGIN), (100,100,255), 1)
            if near_edge:
                # print(f'Near edge ON')
                cv2.putText(display, 'EDGE MODE', (20,450), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)       

        if SHOW_DEPTH_VIEW:
            depth_mm = depth_raw.astype(np.float32) * depth_scale * 1000.0
            depth_center = None

            if chosen is not None:
                depth_center = (int(round(chosen[0])), int(round(chosen[1])))
            depth_clipped = np.clip(depth_mm, DEPTH_MIN_MM, DEPTH_MAX_MM)
            depth_norm = ((depth_clipped - DEPTH_MIN_MM) / (DEPTH_MAX_MM - DEPTH_MIN_MM) * 255).astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            if SHOW_DEPTH_ROI and current_depth_mask is not None:
                ys, xs = np.where(current_depth_mask)
                for px, py in zip(xs, ys):
                    depth_vis[current_depth_y1 + py, current_depth_x1 + px] = (0,255,0)

            draw_depth_legend(depth_vis)
            if chosen is not None:
                cv2.circle(depth_vis, depth_center, 5, (255,255,255), -1)
            cv2.imshow('Depth View', depth_vis)

        if SHOW_MASK:
            cv2.putText(filter_vis, 'GREEN = Accepted', (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(filter_vis, 'RED = Area Reject', (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            cv2.putText(filter_vis, 'ORANGE = Circularity Reject', (10,75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
            if USE_RADIUS_FILTER:
                cv2.putText(filter_vis, 'BLUE = Radius Reject', (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
            cv2.imshow('Final Filter Visualization', filter_vis)
        
        if REFERENCE_MODE == "":
             cv2.putText(display, 'REF MODE: OFF', (WIDTH - 220, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128,128,128), 2)
             send_reference_xy((0,0), 0)

        elif REFERENCE_MODE == "LIVE":
            cv2.putText(display, 'REF MODE: LIVE', (WIDTH - 220, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            ref_mm = reference_pixel_to_mm(ref_pixel)
            send_reference_xy(ref_mm, 1)

            cv2.putText(display, f'Xref: {ref_mm[0]:+6.1f} mm', (WIDTH - 240, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(display, f'Yref: {ref_mm[1]:+6.1f} mm', (WIDTH - 240, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        elif REFERENCE_MODE == "PATH":
            if playback_active and len(ref_path_pixels) > 0:
                color = (0,255,255)
                text = 'REF MODE: PLAYBACK'
                cv2.putText(display, text, (WIDTH - 260, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                ref_pixel = ref_path_pixels[playback_index]

                ref_mm = reference_pixel_to_mm(ref_pixel)
                send_reference_xy(ref_mm, 1)

                cv2.putText(display, f'Xref: {ref_mm[0]:+6.1f} mm', (WIDTH - 240, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                cv2.putText(display, f'Yref: {ref_mm[1]:+6.1f} mm', (WIDTH - 240, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

                playback_index += 1

                if playback_index >= len(ref_path_pixels):
                    playback_index = 0

            elif mouse_down:
                color = (0,0,255)
                text = 'REF MODE: RECORDING'
                cv2.putText(display, text, (WIDTH - 260, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            else:
                color = (255,255,0)
                text = 'REF MODE: READY'
                cv2.putText(display, text, (WIDTH - 260, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                send_reference_xy((0,0),0)

        t_disp_start = time.perf_counter()
        cv2.imshow('Final Code', display)

        key = cv2.waitKey(1) & 0xFF
        t6 = time.perf_counter()

        if PROFILE:

            timing_stats["acquisition"].append((t1 - t0) * 1000)
            timing_stats["detection"].append((t2 - t_det_start) * 1000)
            timing_stats["tracking"].append((t3 - t_track_start) * 1000)
            timing_stats["depth"].append((t4 - t_depth_start) * 1000)
            timing_stats["udp"].append((t5 - t_udp_start) * 1000)
            timing_stats["display"].append((t6 - t_disp_start) * 1000)
            timing_stats["total"].append((t6 - total_start) * 1000)

        if key == ord('z') and z_mm is not None:
            z_reference_mm = z_mm
            print(f'Z reference set to {z_reference_mm:.1f} mm')
        if key == 27:
            break
        if key == ord('l'):
            REFERENCE_MODE = "LIVE"
            print("LIVE REFERENCE MODE")

        elif key == ord('r'):
            REFERENCE_MODE = "PATH"
            playback_active = False
            print("PATH DRAW MODE")

        elif key == ord(' '):
            if len(ref_path_pixels) > 0:
                playback_active = True
                playback_index = 0
                print("PATH PLAYBACK")

        elif key == ord('c'):
            ref_path_pixels.clear()
            playback_active = False
            print("Trajectory cleared")

        elif key == ord('n'):
            REFERENCE_MODE = ""
            playback_active = False
            print("REFERENCE DISABLED")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    
    if PROFILE:

        print("\n================ FINAL SYSTEM REPORT ================")
        for key, values in timing_stats.items():
            if len(values):
                print(
                    f"{key:12s}: "
                    f"avg = {np.mean(values):7.2f} ms | "
                    f"max = {np.max(values):7.2f} ms | "
                    f"min = {np.min(values):7.2f} ms"
                )
        print("=====================================================")