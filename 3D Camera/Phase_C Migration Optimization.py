import socket
import cv2
import numpy as np
import time
import pyrealsense2 as rs
import os

# ================= CONFIG =================
serverAddressPort = ("192.168.140.8", 49001)
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

WIDTH = 640
HEIGHT = 480
FPS = 60

angle = -0.65
center = (WIDTH // 2, HEIGHT // 2)
M = cv2.getRotationMatrix2D(center, angle, 1.0)

threshold_value = 100
min_width = 20
max_width = 60
ROI_RADIUS = 80
MAX_FRAME_FAILS = 20

# Physical plate bounds (user tuned)
PLATE_X1 = 100
PLATE_Y1 = 20
PLATE_X2 = 530
PLATE_Y2 = 450

# Safe controllable workspace
SAFE_MARGIN = 40
SAFE_X1 = PLATE_X1 + SAFE_MARGIN
SAFE_Y1 = PLATE_Y1 + SAFE_MARGIN
SAFE_X2 = PLATE_X2 - SAFE_MARGIN
SAFE_Y2 = PLATE_Y2 - SAFE_MARGIN

DEBUG_VISUAL = True
DEBUG_PRINT_UDP = False
PROFILE = True
RECORD_VIDEO = True
RECORD_DURATION_SEC = 30
RECORD_PATH = r"C:\Users\alexi\Desktop\Master_S&C\13. (5SC26) Systems and Control Integration Project\Project\BallANDPlate\3D camera\Recordings"
RECORD_BASENAME = "phase_c_recording"

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
pipeline.start(config)

print(f"RealSense initialized: {WIDTH}x{HEIGHT} @ {FPS} FPS")

video_writer = None
if RECORD_VIDEO:
    os.makedirs(RECORD_PATH, exist_ok=True)
    existing = [f for f in os.listdir(RECORD_PATH) if f.startswith(RECORD_BASENAME) and f.endswith('.avi')]
    next_idx = 1
    if existing:
        indices = []
        for f in existing:
            stem = f.replace('.avi', '')
            suffix = stem.replace(RECORD_BASENAME + '_', '')
            if suffix.isdigit():
                indices.append(int(suffix))
        if indices:
            next_idx = max(indices) + 1
    filename = f"{RECORD_BASENAME}_{next_idx:03d}.avi"
    full_record_path = os.path.join(RECORD_PATH, filename)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video_writer = cv2.VideoWriter(full_record_path, fourcc, FPS, (WIDTH, HEIGHT))
    print(f"Recording enabled -> {full_record_path}")

stats = {k: [] for k in ["acquisition", "rotation", "tracking", "udp", "display", "total"]}


def dist(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def send_ball_position_to_microlab(chosen):
    x = int(chosen[0])
    y = int(chosen[1])
    x_bytes = x.to_bytes(2, byteorder='little')
    y_bytes = y.to_bytes(2, byteorder='little')
    packet = bytearray([y_bytes[0], y_bytes[1], 0, 0, x_bytes[0], x_bytes[1]])
    if DEBUG_PRINT_UDP:
        print(f"UDP -> X={x}, Y={y}")
    UDPClientSocket.sendto(packet, serverAddressPort)


def detect_ball_houghcircles(gray):
    blur = cv2.GaussianBlur(gray, (17, 17), 0)
    return cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=1.6,
        minDist=300,
        param1=100,
        param2=40,
        minRadius=10,
        maxRadius=200,
    )


def detect_ball_custom_roi(gray, prev_circle):
    if prev_circle is None:
        return None, None

    cx = int(prev_circle[0])
    cy = int(prev_circle[1])

    # If prediction exits safe workspace, force reacquisition
    if cx < SAFE_X1 or cx > SAFE_X2 or cy < SAFE_Y1 or cy > SAFE_Y2:
        return None, None

    x1 = max(cx - ROI_RADIUS, SAFE_X1)
    x2 = min(cx + ROI_RADIUS, SAFE_X2)
    y1 = max(cy - ROI_RADIUS, SAFE_Y1)
    y2 = min(cy + ROI_RADIUS, SAFE_Y2)

    if x2 <= x1 or y2 <= y1:
        return None, None

    roi = gray[y1:y2, x1:x2]
    local_x = np.clip(cx - x1, 0, roi.shape[1] - 1)
    local_y = np.clip(cy - y1, 0, roi.shape[0] - 1)

    if roi[local_y, local_x] >= threshold_value:
        return None, (x1, y1, x2, y2)

    flooded = roi.copy()
    mask = np.zeros((roi.shape[0] + 2, roi.shape[1] + 2), np.uint8)
    cv2.floodFill(flooded, mask, (local_x, local_y), 255, loDiff=2, upDiff=2)
    _, thresh = cv2.threshold(flooded, 254, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, (x1, y1, x2, y2)

    ball_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(ball_contour)

    if not (min_width < w < max_width):
        return None, (x1, y1, x2, y2)

    gx = x + x1 + w // 2
    gy = y + y1 + h // 2

    if gx < SAFE_X1 or gx > SAFE_X2 or gy < SAFE_Y1 or gy > SAFE_Y2:
        return None, (x1, y1, x2, y2)

    return (gx, gy, w, h), (x1, y1, x2, y2)


def track_ball(gray, prev_circle=None, use_hough=True):
    roi_box = None
    if use_hough:
        circles = detect_ball_houghcircles(gray)
        if circles is None:
            return None, True, roi_box
        circles = np.uint16(np.around(circles))
        chosen = None
        for c in circles[0, :]:
            if SAFE_X1 <= c[0] <= SAFE_X2 and SAFE_Y1 <= c[1] <= SAFE_Y2:
                if chosen is None:
                    chosen = c
                elif prev_circle is not None and dist(c[0], c[1], prev_circle[0], prev_circle[1]) < dist(chosen[0], chosen[1], prev_circle[0], prev_circle[1]):
                    chosen = c
        if chosen is None:
            return None, True, roi_box
        return chosen, False, roi_box
    else:
        chosen, roi_box = detect_ball_custom_roi(gray, prev_circle)
        if chosen is None:
            return None, True, roi_box
        return chosen, False, roi_box


prev_circle = None
ref_circle = None
dx = None
dy = None
use_hough = True
frame_fail_count = 0

try:
    record_start_time = time.perf_counter()
    while True:
        total_start = time.perf_counter()
        t0 = time.perf_counter()

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            frame_fail_count += 1
            if frame_fail_count >= MAX_FRAME_FAILS:
                print("Too many frame failures")
                break
            continue

        frame_fail_count = 0
        frame = np.asanyarray(color_frame.get_data())
        t1 = time.perf_counter()

        frame = cv2.warpAffine(frame, M, (WIDTH, HEIGHT))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        gray[:PLATE_Y1, :] = 255
        gray[PLATE_Y2:, :] = 255
        gray[:, :PLATE_X1] = 255
        gray[:, PLATE_X2:] = 255
        t2 = time.perf_counter()

        if prev_circle is not None and dx is not None and dy is not None:
            ref_circle = (int(prev_circle[0] + dx), int(prev_circle[1] + dy))
        else:
            ref_circle = prev_circle

        chosen, use_hough, roi_box = track_ball(gray, ref_circle, use_hough)
        t3 = time.perf_counter()

        if chosen is not None:
            if prev_circle is not None:
                dx = int(chosen[0]) - int(prev_circle[0])
                dy = int(chosen[1]) - int(prev_circle[1])
            prev_circle = chosen
        else:
            dx = dy = None
            prev_circle = None
            ref_circle = None

        t_udp = time.perf_counter()
        if chosen is not None and not use_hough:
            send_ball_position_to_microlab(chosen)
        t4 = time.perf_counter()

        if DEBUG_VISUAL:
            fps_est = 1.0 / max((t4 - total_start), 1e-6)
            cv2.rectangle(frame, (PLATE_X1, PLATE_Y1), (PLATE_X2, PLATE_Y2), (0, 255, 255), 2)
            cv2.rectangle(frame, (SAFE_X1, SAFE_Y1), (SAFE_X2, SAFE_Y2), (0, 0, 255), 2)
            cv2.putText(frame, f"FPS: {fps_est:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            mode = "Hough" if use_hough else "Tracking"
            color = (0,255,0) if use_hough else (255,0,255)
            cv2.putText(frame, f"Mode: {mode}", (20,80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            if ref_circle is not None:
                cv2.circle(frame, (int(ref_circle[0]), int(ref_circle[1])), 5, (255,255,0), 2)
            if roi_box is not None:
                x1, y1, x2, y2 = roi_box
                cv2.rectangle(frame, (x1,y1), (x2,y2), (255,255,0), 1)
            if chosen is not None:
                cv2.circle(frame, (chosen[0], chosen[1]), 4, (0,0,255), -1)
                if len(chosen) >= 3:
                    cv2.circle(frame, (chosen[0], chosen[1]), int(chosen[2]/2), color, 2)
                cv2.putText(frame, f"X:{chosen[0]} Y:{chosen[1]}", (20,120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)

            if RECORD_VIDEO and video_writer is not None:
                video_writer.write(frame)

            cv2.imshow("Phase C Optimized Safe Workspace", frame)

            if RECORD_VIDEO and (time.perf_counter() - record_start_time) >= RECORD_DURATION_SEC:
                print(f"Recording complete ({RECORD_DURATION_SEC} s). Stopping automatically.")
                break
            if cv2.waitKey(1) & 0xFF == 27:
                break

        t5 = time.perf_counter()
        if PROFILE:
            stats["acquisition"].append((t1 - t0) * 1000)
            stats["rotation"].append((t2 - t1) * 1000)
            stats["tracking"].append((t3 - t2) * 1000)
            stats["udp"].append((t4 - t_udp) * 1000)
            stats["display"].append((t5 - t4) * 1000)
            stats["total"].append((t5 - total_start) * 1000)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    if video_writer is not None:
        video_writer.release()
    UDPClientSocket.close()
    if PROFILE:
        print("\n================ TIMING REPORT ================")
        for k, v in stats.items():
            if v:
                print(f"{k:12s}: avg={np.mean(v):7.2f} ms | max={np.max(v):7.2f} ms | min={np.min(v):7.2f} ms")
        print("===============================================")
