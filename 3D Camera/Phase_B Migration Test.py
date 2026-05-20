import socket
import cv2
import numpy as np
import time
import pyrealsense2 as rs

# ============================================================
# UDP CONFIG
# ============================================================

serverAddressPort = ("192.168.140.8", 49001)
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# ============================================================
# REALSENSE CONFIG
# ============================================================

WIDTH = 640
HEIGHT = 480
FPS = 60

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    WIDTH,
    HEIGHT,
    rs.format.bgr8,
    FPS
)

pipeline.start(config)

print(f"RealSense initialized: {WIDTH}x{HEIGHT} @ {FPS} FPS")

# ============================================================
# CALIBRATION / PARAMETERS
# ============================================================

angle = -0.65
center = (WIDTH // 2, HEIGHT // 2)
M = cv2.getRotationMatrix2D(center, angle, 1.0)

threshold_value = 100
min_width = 20
max_width = 60

DEBUG_VISUAL = True

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

    UDPClientSocket.sendto(bytesToSend, serverAddressPort)

# ============================================================
# DETECTION
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

            if DEBUG_VISUAL:
                cv2.circle(
                    frame,
                    (chosen[0], chosen[1]),
                    chosen[2],
                    (0, 255, 0),
                    2
                )

                cv2.circle(
                    frame,
                    (chosen[0], chosen[1]),
                    4,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    frame,
                    "Mode: Hough",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

            return chosen, False

        else:
            if DEBUG_VISUAL:
                cv2.putText(
                    frame,
                    "NO BALL FOUND",
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
                    int(chosen[2] / 2),
                    (255, 0, 255),
                    2
                )

                cv2.circle(
                    frame,
                    (chosen[0], chosen[1]),
                    4,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    frame,
                    "Mode: Tracking",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 0, 255),
                    2
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
        loop_start = time.perf_counter()

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        frame = cv2.warpAffine(frame, M, (WIDTH, HEIGHT))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if prev_circle is not None and dx is not None and dy is not None:
            ref_circle = (
                int(prev_circle[0] + dx),
                int(prev_circle[1] + dy)
            )
        else:
            ref_circle = prev_circle

        if ref_circle is not None and DEBUG_VISUAL:
            cv2.circle(
                frame,
                (int(ref_circle[0]), int(ref_circle[1])),
                6,
                (255, 255, 0),
                2
            )

        chosen, use_hough = track_ball(
            frame,
            gray,
            ref_circle,
            use_hough
        )

        if chosen is not None:
            if prev_circle is not None:
                dx = int(chosen[0]) - int(prev_circle[0])
                dy = int(chosen[1]) - int(prev_circle[1])

            prev_circle = chosen

            send_ball_position_to_microlab(chosen)

        else:
            dx = None
            dy = None
            prev_circle = None
            ref_circle = None

        if chosen is not None and DEBUG_VISUAL:
            cv2.putText(
                frame,
                f"X: {chosen[0]}  Y: {chosen[1]}",
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 0),
                2
            )

        elapsed = time.perf_counter() - loop_start
        fps = 1.0 / max(elapsed, 1e-6)

        cv2.putText(
            frame,
            f"FPS: {fps:.1f}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        cv2.imshow("Phase B - D435 Migration Debug", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    UDPClientSocket.close()