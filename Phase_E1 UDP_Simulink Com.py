import socket
import cv2
import numpy as np
import time
import pyrealsense2 as rs
import os
import math

# ================= CONFIG =================
serverAddressPort = ("192.168.140.8", 49001)
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

WIDTH = 640
HEIGHT = 480
FPS = 60

angle = -0.85
center = (WIDTH // 2, HEIGHT // 2)
M = cv2.getRotationMatrix2D(center, angle, 1.0)

# Workspace (user tuned)
PLATE_X1 = 80
PLATE_Y1 = 20
PLATE_X2 = 530
PLATE_Y2 = 460

SAFE_MARGIN = 50
SAFE_X1 = PLATE_X1 + SAFE_MARGIN
SAFE_Y1 = PLATE_Y1 + SAFE_MARGIN
SAFE_X2 = PLATE_X2 - SAFE_MARGIN
SAFE_Y2 = PLATE_Y2 - SAFE_MARGIN

PLATE_CENTER_X = int((PLATE_X1 + PLATE_X2) / 2)
PLATE_CENTER_Y = int((PLATE_Y1 + PLATE_Y2) / 2)

# Physical calibration
PLATE_SIZE_MM = 400.0
GRID_SPACING_MM = 10.0
MAJOR_GRID_MM = 50.0
SCALE_X = PLATE_SIZE_MM / (PLATE_X2 - PLATE_X1)
SCALE_Y = PLATE_SIZE_MM / (PLATE_Y2 - PLATE_Y1)

# Detection parameters
THRESHOLD = 75
MIN_AREA = 200
MAX_AREA = 4000
MIN_CIRCULARITY = 0.45
MAX_TRACK_DISTANCE = 120

# Recording
RECORD_VIDEO = False
RECORD_DURATION_SEC = 30
RECORD_PATH = r"C:\Users\alexi\Desktop\Master_S&C\13. (5SC26) Systems and Control Integration Project\Project\BallANDPlate\3D camera\Recordings"
RECORD_BASENAME = "phase_e1_recording"

# Runtime
MAX_FRAME_FAILS = 20
DEBUG_VISUAL = True
DEBUG_PRINT_UDP = True
SHOW_MASK = True
PROFILE = True

# ================= HELPERS =================
def pixel_to_control_coords(pixel_xy):
    x_ctrl = (pixel_xy[0] - PLATE_CENTER_X) * SCALE_X
    y_ctrl = (PLATE_CENTER_Y - pixel_xy[1]) * SCALE_Y
    return x_ctrl, y_ctrl


def draw_metric_grid(frame):
    for mm in range(-200, 201, int(GRID_SPACING_MM)):
        x = int(PLATE_CENTER_X + mm / SCALE_X)
        y = int(PLATE_CENTER_Y - mm / SCALE_Y)

        if PLATE_X1 <= x <= PLATE_X2:
            thickness = 2 if mm % int(MAJOR_GRID_MM) == 0 else 1
            cv2.line(frame, (x, PLATE_Y1), (x, PLATE_Y2), (100, 100, 100), thickness)

        if PLATE_Y1 <= y <= PLATE_Y2:
            thickness = 2 if mm % int(MAJOR_GRID_MM) == 0 else 1
            cv2.line(frame, (PLATE_X1, y), (PLATE_X2, y), (100, 100, 100), thickness)

    for mm in range(-200, 201, 50):
        x = int(PLATE_CENTER_X + mm / SCALE_X)
        y = int(PLATE_CENTER_Y - mm / SCALE_Y)

        if PLATE_X1 <= x <= PLATE_X2:
            cv2.putText(frame, f'{mm}', (x - 12, PLATE_CENTER_Y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

        if PLATE_Y1 <= y <= PLATE_Y2 and mm != 0:
            cv2.putText(frame, f'{mm}', (PLATE_CENTER_X + 6, y + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)


def send_ball_position_mm(ctrl_xy):
    x_mm = int(ctrl_xy[0])
    y_mm = int(ctrl_xy[1])

    x_bytes = x_mm.to_bytes(2, byteorder='little', signed=True)
    y_bytes = y_mm.to_bytes(2, byteorder='little', signed=True)

    packet = bytearray([
        y_bytes[0], y_bytes[1],
        0, 0,
        x_bytes[0], x_bytes[1]
    ])
    if DEBUG_PRINT_UDP:
        print(f"UDP -> X={x_mm}, Y={y_mm}")
    UDPClientSocket.sendto(packet, serverAddressPort)


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
    cx = int(Mmom['m10'] / Mmom['m00'])
    cy = int(Mmom['m01'] / Mmom['m00'])
    return (cx, cy)


def euclidean_distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def detect_ball(gray, predicted=None):
    _, mask = cv2.threshold(gray, THRESHOLD, 255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((5, 5), np.uint8)
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

        if not (SAFE_X1 <= ctr[0] <= SAFE_X2 and SAFE_Y1 <= ctr[1] <= SAFE_Y2):
            continue

        candidates.append((contour, ctr, area, circ))

    if not candidates:
        return None, mask

    if predicted is None:
        best = max(candidates, key=lambda x: x[2])
        return best, mask

    valid = []
    for candidate in candidates:
        dist = euclidean_distance(candidate[1], predicted)
        if dist <= MAX_TRACK_DISTANCE:
            valid.append((candidate, dist))

    if valid:
        valid.sort(key=lambda x: x[1])
        return valid[0][0], mask

    best = max(candidates, key=lambda x: x[2])
    return best, mask


# ================= CAMERA INIT =================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.y8, FPS)
pipeline.start(config)

video_writer = None
if RECORD_VIDEO:
    os.makedirs(RECORD_PATH, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video_writer = cv2.VideoWriter(os.path.join(RECORD_PATH, RECORD_BASENAME + '.avi'), fourcc, FPS, (WIDTH, HEIGHT))

print(f"RealSense initialized: {WIDTH}x{HEIGHT} @ {FPS} FPS")

stats = {k: [] for k in ['acquisition', 'rotation', 'tracking', 'udp', 'display', 'total']}

# ================= MAIN LOOP =================
prev_pos = None
velocity = None
frame_fail_count = 0
record_start = time.perf_counter()

try:
    while True:
        total_start = time.perf_counter()
        t0 = time.perf_counter()

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            frame_fail_count += 1
            if frame_fail_count >= MAX_FRAME_FAILS:
                break
            continue

        frame_fail_count = 0
        frame = np.asanyarray(color_frame.get_data())
        t1 = time.perf_counter()

        frame = cv2.warpAffine(frame, M, (WIDTH, HEIGHT))
        frame = cv2.flip(frame, -1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        gray[:PLATE_Y1, :] = 255
        gray[PLATE_Y2:, :] = 255
        gray[:, :PLATE_X1] = 255
        gray[:, PLATE_X2:] = 255

        t2 = time.perf_counter()

        predicted = None
        if prev_pos is not None and velocity is not None:
            predicted = (int(prev_pos[0] + velocity[0]), int(prev_pos[1] + velocity[1]))

        result, mask = detect_ball(gray, predicted)
        t3 = time.perf_counter()

        chosen = None
        contour = None
        ctrl_coords = None

        if result is not None:
            contour, chosen, area, circ = result
            ctrl_coords = pixel_to_control_coords(chosen)

            if prev_pos is not None:
                velocity = (chosen[0] - prev_pos[0], chosen[1] - prev_pos[1])
            else:
                velocity = (0, 0)

            prev_pos = chosen
        else:
            prev_pos = None
            velocity = None

        t_udp = time.perf_counter()
        if ctrl_coords is not None:
            send_ball_position_mm(ctrl_coords)
        t4 = time.perf_counter()

        if DEBUG_VISUAL:
            display_frame = frame.copy()
            draw_metric_grid(display_frame)

            cv2.rectangle(display_frame, (PLATE_X1, PLATE_Y1), (PLATE_X2, PLATE_Y2), (0, 255, 255), 2)
            cv2.rectangle(display_frame, (SAFE_X1, SAFE_Y1), (SAFE_X2, SAFE_Y2), (0, 0, 255), 2)

            if contour is not None:
                cv2.drawContours(display_frame, [contour], -1, (255, 0, 255), 2)

            if chosen is not None:
                cv2.circle(display_frame, chosen, 5, (0, 0, 255), -1)

            if ctrl_coords is not None:
                cv2.putText(display_frame,
                            f'x:{ctrl_coords[0]:+6.1f} mm   y:{ctrl_coords[1]:+6.1f} mm',
                            (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (255, 255, 0),
                            2)

            cv2.imshow('Phase E1 UDP mm Tracker', display_frame)
            if SHOW_MASK:
                cv2.imshow('Binary Mask', mask)

            if cv2.waitKey(1) & 0xFF == 27:
                break

finally:
    pipeline.stop()
    if video_writer is not None:
        video_writer.release()
    cv2.destroyAllWindows()
    UDPClientSocket.close()
