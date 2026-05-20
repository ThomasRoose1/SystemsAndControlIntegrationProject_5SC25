import socket
import cv2
import numpy as np
import time

# ============================================================
# CONFIGURATION
# ============================================================

# UDP configuration
serverAddressPort = ("192.168.140.8", 49001)
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Camera configuration (MATCH D435 SETTINGS)
WIDTH = 640
HEIGHT = 480
FPS = 30
frame_fail_count = 0
MAX_FRAME_FAILS = 20

# Rotation calibration
angle = -0.65
center = (WIDTH // 2, HEIGHT // 2)
M = cv2.getRotationMatrix2D(center, angle, 1.0)

# Detection parameters (MATCH PHASE C)
threshold_value = 100
min_width = 20
max_width = 60

# Debug / profiling
DEBUG_VISUAL = True
DEBUG_PRINT_UDP = False
PROFILE = True

# ============================================================
# WEBCAM INITIALIZATION
# ============================================================

video = cv2.VideoCapture(1, cv2.CAP_DSHOW)

video.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
video.set(cv2.CAP_PROP_FPS, FPS)

print("Opened:", video.isOpened())
print("Backend:", video.getBackendName())
print("Width:", video.get(cv2.CAP_PROP_FRAME_WIDTH))
print("Height:", video.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("FPS:", video.get(cv2.CAP_PROP_FPS))

actual_width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
actual_fps = video.get(cv2.CAP_PROP_FPS)

print(f"Webcam initialized: {actual_width}x{actual_height} @ {actual_fps:.1f} FPS")

# ============================================================
# TIMING STORAGE
# ============================================================

timing_stats = {
    "acquisition": [],
    "rotation": [],
    "tracking": [],
    "udp": [],
    "display": [],
    "total": []
}

# ============================================================
# HELPERS
# ============================================================

def dist(x1, y1, x2, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5


def send_ball_position_to_microlab(chosen):
    x = int(chosen[0])
    y = int(chosen[1])

    x_bytes = x.to_bytes(2, byteorder='little')
    y_bytes = y.to_bytes(2, byteorder='little')

    bytesToSend = bytearray([
        y_bytes[0], y_bytes[1],
        0, 0,
        x_bytes[0], x_bytes[1]
    ])

    if DEBUG_PRINT_UDP:
        print(f"UDP -> X={x}, Y={y}")

    UDPClientSocket.sendto(bytesToSend, serverAddressPort)

# ============================================================
# DETECTION FUNCTIONS
# ============================================================

def detect_ball_houghcircles(gray):
    blur = cv2.GaussianBlur(gray, (17, 17), 0)

    circles = cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=1.6,
        minDist=300,
        param1=100,
        param2=40,
        minRadius=10,
        maxRadius=200
    )

    return circles


def detect_ball_custom(gray, prev_circle):
    if prev_circle is None:
        return None

    x = int(prev_circle[0])
    y = int(prev_circle[1])

    x = np.clip(x, 0, gray.shape[1] - 1)
    y = np.clip(y, 0, gray.shape[0] - 1)

    pixel_val = gray[y, x]

    if pixel_val >= threshold_value:
        return None

    flooded = gray.copy()
    mask = np.zeros((gray.shape[0] + 2, gray.shape[1] + 2), np.uint8)

    cv2.floodFill(
        flooded,
        mask,
        seedPoint=(x, y),
        newVal=255,
        loDiff=2,
        upDiff=2
    )

    _, thresh = cv2.threshold(
        flooded,
        254,
        255,
        cv2.THRESH_BINARY
    )

    contours, _ = cv2.findContours(
        thresh,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    ball_contour = max(contours, key=cv2.contourArea)

    x, y, w, h = cv2.boundingRect(ball_contour)

    if min_width < w < max_width:
        return (x + w // 2, y + h // 2, w, h)

    return None


def track_ball(frame, gray, prev_circle=None, use_hough=True):
    if use_hough:
        circles = detect_ball_houghcircles(gray)

        if circles is not None:
            circles = np.uint16(np.around(circles))

            chosen = circles[0, 0]

            for j in circles[0, :]:
                if prev_circle is not None:
                    d_current = dist(
                        chosen[0], chosen[1],
                        prev_circle[0], prev_circle[1]
                    )

                    d_candidate = dist(
                        j[0], j[1],
                        prev_circle[0], prev_circle[1]
                    )

                    if d_candidate < d_current:
                        chosen = j

            return chosen, False

        else:
            if DEBUG_VISUAL:
                cv2.putText(
                    frame,
                    'No ball found',
                    (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    2
                )

            return None, True

    else:
        chosen = detect_ball_custom(gray, prev_circle)

        if chosen is not None:
            if DEBUG_VISUAL:
                cv2.circle(
                    frame,
                    (chosen[0], chosen[1]),
                    1,
                    (0, 100, 100),
                    3
                )

                cv2.circle(
                    frame,
                    (chosen[0], chosen[1]),
                    int(chosen[2] / 2),
                    (255, 0, 255),
                    3
                )

            return chosen, False

        return None, True

# ============================================================
# MAIN LOOP
# ============================================================

prev_circle = None
ref_circle = None
dx = None
dy = None
use_hough = True

try:
    while True:
        total_start = time.perf_counter()

        # ----------------------------------------------------
        # Acquisition
        # ----------------------------------------------------
        t0 = time.perf_counter()

        ret, frame = video.read()

        if not ret or frame is None:
            frame_fail_count += 1
            print(f"Warning: dropped frame ({frame_fail_count})")

            if frame_fail_count >= MAX_FRAME_FAILS:
                print("Too many consecutive frame failures. Exiting.")
                break

            continue

        frame_fail_count = 0

        t1 = time.perf_counter()

        # ----------------------------------------------------
        # Rotation + grayscale
        # ----------------------------------------------------
        # frame = cv2.warpAffine(frame, M, (WIDTH, HEIGHT))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        t2 = time.perf_counter()

        # ----------------------------------------------------
        # Prediction
        # ----------------------------------------------------
        if prev_circle is not None and dx is not None and dy is not None:
            ref_circle = (
                int(prev_circle[0] + dx),
                int(prev_circle[1] + dy)
            )
        else:
            ref_circle = prev_circle

        # ----------------------------------------------------
        # Tracking
        # ----------------------------------------------------
        chosen, use_hough = track_ball(
            frame,
            gray,
            ref_circle,
            use_hough
        )

        t3 = time.perf_counter()

        # ----------------------------------------------------
        # Update tracking state
        # ----------------------------------------------------
        if chosen is not None:
            if prev_circle is not None:
                dx = int(chosen[0]) - int(prev_circle[0])
                dy = int(chosen[1]) - int(prev_circle[1])

            prev_circle = chosen

        else:
            dx = None
            dy = None
            prev_circle = None
            ref_circle = None

        # ----------------------------------------------------
        # UDP send
        # ----------------------------------------------------
        t_udp_start = time.perf_counter()

        if chosen is not None and not use_hough:
            send_ball_position_to_microlab(chosen)

        t4 = time.perf_counter()

        # ----------------------------------------------------
        # Display
        # ----------------------------------------------------
        if DEBUG_VISUAL:
            fps_est = 1.0 / max((t4 - total_start), 1e-6)

            cv2.putText(
                frame,
                f"FPS: {fps_est:.1f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            cv2.imshow("Webcam Baseline Tracking", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break

        t5 = time.perf_counter()

        # ----------------------------------------------------
        # Timing logging
        # ----------------------------------------------------
        if PROFILE:
            timing_stats["acquisition"].append((t1 - t0) * 1000)
            timing_stats["rotation"].append((t2 - t1) * 1000)
            timing_stats["tracking"].append((t3 - t2) * 1000)
            timing_stats["udp"].append((t4 - t_udp_start) * 1000)
            timing_stats["display"].append((t5 - t4) * 1000)
            timing_stats["total"].append((t5 - total_start) * 1000)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    video.release()
    cv2.destroyAllWindows()
    UDPClientSocket.close()

    if PROFILE:
        print("\n================ WEBCAM BASELINE REPORT ================")

        for key, values in timing_stats.items():
            if len(values) > 0:
                print(
                    f"{key:12s}: "
                    f"avg = {np.mean(values):7.2f} ms | "
                    f"max = {np.max(values):7.2f} ms | "
                    f"min = {np.min(values):7.2f} ms"
                )

        print("=======================================================")