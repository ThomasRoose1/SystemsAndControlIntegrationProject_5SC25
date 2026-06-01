import cv2
import numpy as np
import pyrealsense2 as rs
import math
import os

# =====================================================
# CONFIGURATION
# =====================================================

WIDTH = 640
HEIGHT = 480
FPS = 60

THRESHOLD = 75

MIN_AREA = 200
MAX_AREA = 2000

MIN_CIRCULARITY = 0.8

SAVE_FOLDER = r"C:\git\SystemsAndControlIntegrationProject_5SC25\3D Camera\Figures"

os.makedirs(SAVE_FOLDER, exist_ok=True)

# =====================================================
# CALIBRATED PLATE CONTOUR
# =====================================================

PLATE_QUAD = np.array([
    [94, 45],
    [502, 44],
    [506, 453],
    [92, 456]
], dtype=np.int32)

# =====================================================
# HELPERS
# =====================================================

def contour_circularity(contour):

    area = cv2.contourArea(contour)

    perimeter = cv2.arcLength(contour, True)

    if perimeter == 0:
        return 0

    return 4 * math.pi * area / (perimeter ** 2)


def contour_centroid(contour):

    M = cv2.moments(contour)

    if M["m00"] == 0:
        return None

    return (
        int(M["m10"] / M["m00"]),
        int(M["m01"] / M["m00"])
    )


def draw_plate_region(img):
    return img.copy()


# =====================================================
# REALSENSE
# =====================================================

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

# =====================================================
# WINDOWS
# =====================================================

WINDOW_W = 640
WINDOW_H = 480

cv2.namedWindow("1 RGB", cv2.WINDOW_NORMAL)
cv2.namedWindow("2 Threshold", cv2.WINDOW_NORMAL)
cv2.namedWindow("3 All Contours", cv2.WINDOW_NORMAL)
cv2.namedWindow("4 Area Filter", cv2.WINDOW_NORMAL)
cv2.namedWindow("5 Circularity Filter", cv2.WINDOW_NORMAL)
cv2.namedWindow("6 Final Detection", cv2.WINDOW_NORMAL)

cv2.resizeWindow("1 RGB", WINDOW_W, WINDOW_H)
cv2.resizeWindow("2 Threshold", WINDOW_W, WINDOW_H)
cv2.resizeWindow("3 All Contours", WINDOW_W, WINDOW_H)
cv2.resizeWindow("4 Area Filter", WINDOW_W, WINDOW_H)
cv2.resizeWindow("5 Circularity Filter", WINDOW_W, WINDOW_H)
cv2.resizeWindow("6 Final Detection", WINDOW_W, WINDOW_H)

# Optional automatic layout

cv2.moveWindow("1 RGB", 0, 0)
cv2.moveWindow("2 Threshold", 650, 0)
cv2.moveWindow("3 All Contours", 1320, 0)
cv2.moveWindow("4 Area Filter", 0, 520)
cv2.moveWindow("5 Circularity Filter", 650, 520)
cv2.moveWindow("6 Final Detection", 1300, 520)

# =====================================================
# FREEZE FEATURE
# =====================================================

paused = False
frozen_frame = None

# =====================================================
# MAIN LOOP
# =====================================================

try:

    while True:

        # =============================================
        # FRAME ACQUISITION
        # =============================================

        if not paused:

            frames = pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            frame = np.asanyarray(
                color_frame.get_data()
            )

            frozen_frame = frame.copy()

        else:

            frame = frozen_frame.copy()

        # =============================================
        # APPLY PLATE ROI IMMEDIATELY
        # =============================================
        frame = cv2.flip(frame, -1)

        frame_roi = frame.copy()

        # =============================================
        # GRAYSCALE
        # =============================================

        gray = cv2.cvtColor(
            frame_roi,
            cv2.COLOR_BGR2GRAY
        )

        # =============================================
        # THRESHOLD
        # =============================================

        _, thresh = cv2.threshold(
            gray,
            THRESHOLD,
            255,
            cv2.THRESH_BINARY_INV
        )

        # =============================================
        # MORPHOLOGY
        # =============================================

        kernel = np.ones(
            (5, 5),
            np.uint8
        )

        opened = cv2.morphologyEx(
            thresh,
            cv2.MORPH_OPEN,
            kernel
        )

        closed = cv2.morphologyEx(
            opened,
            cv2.MORPH_CLOSE,
            kernel
        )

        # =============================================
        # ALL CONTOURS
        # =============================================

        contours, _ = cv2.findContours(
            closed,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        all_contours_img = np.zeros_like(frame_roi)

        cv2.drawContours(
            all_contours_img,
            contours,
            -1,
            (255,255,255),
            2
        )

        cv2.putText(
            all_contours_img,
            f"Contours Found: {len(contours)}",
            (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255,255,255),
            2
        )

        # =============================================
        # AREA FILTER
        # =============================================

        area_filtered = []

        area_img = np.zeros_like(frame_roi)

        for contour in contours:

            area = cv2.contourArea(contour)

            if MIN_AREA < area < MAX_AREA:

                area_filtered.append(contour)

                # ACCEPTED CONTOURS
                cv2.drawContours(
                    area_img,
                    [contour],
                    -1,
                    (0,255,0),
                    3
                )

            else:

                # REJECTED CONTOURS
                cv2.drawContours(
                    area_img,
                    [contour],
                    -1,
                    (0,0,255),
                    2
                )

        cv2.putText(
            area_img,
            f"Accepted: {len(area_filtered)}",
            (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0,255,0),
            2
        )

        cv2.putText(
            area_img,
            "Green = Accepted",
            (10,60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0,255,0),
            2
        )

        cv2.putText(
            area_img,
            "Red = Rejected",
            (10,90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0,0,255),
            2
        )

        # =============================================
        # CIRCULARITY FILTER
        # =============================================

        circ_filtered = []

        circ_img = np.zeros_like(frame_roi)

        for contour in area_filtered:

            circ = contour_circularity(contour)

            if circ > MIN_CIRCULARITY:

                circ_filtered.append(contour)

                # ACCEPTED
                cv2.drawContours(
                    circ_img,
                    [contour],
                    -1,
                    (0,255,0),
                    3
                )

                ctr = contour_centroid(contour)

                if ctr is not None:

                    cv2.putText(
                        circ_img,
                        f"{circ:.2f}",
                        (ctr[0] + 10, ctr[1]),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0,255,0),
                        1
                    )

            else:

                # REJECTED
                cv2.drawContours(
                    circ_img,
                    [contour],
                    -1,
                    (0,0,255),
                    2
                )

                ctr = contour_centroid(contour)

                if ctr is not None:

                    cv2.putText(
                        circ_img,
                        f"{circ:.2f}",
                        (ctr[0] + 10, ctr[1]),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0,0,255),
                        1
                    )

        cv2.putText(
            circ_img,
            f"Accepted: {len(circ_filtered)}",
            (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0,255,0),
            2
        )

        cv2.putText(
            circ_img,
            "Green = Accepted",
            (10,60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0,255,0),
            2
        )

        cv2.putText(
            circ_img,
            "Red = Rejected",
            (10,90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0,0,255),
            2
        )

        # =============================================
        # FINAL DETECTION
        # =============================================

        final_img = draw_plate_region(
            frame_roi
        )

        if circ_filtered:

            best_ball = max(
                circ_filtered,
                key=cv2.contourArea
            )

            center = contour_centroid(
                best_ball
            )

            if center is not None:

                area = cv2.contourArea(
                    best_ball
                )

                circ = contour_circularity(
                    best_ball
                )

                cv2.drawContours(
                    final_img,
                    [best_ball],
                    -1,
                    (255,0,255),
                    3
                )

                cv2.circle(
                    final_img,
                    center,
                    6,
                    (0,0,255),
                    -1
                )

                cv2.putText(
                    final_img,
                    f"Area = {area:.0f}",
                    (20,40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0,255,0),
                    2
                )

                cv2.putText(
                    final_img,
                    f"Circularity = {circ:.2f}",
                    (20,70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255,0,255),
                    2
                )

                cv2.putText(
                    final_img,
                    f"Center = ({center[0]}, {center[1]})",
                    (20,100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255,255,0),
                    2
                )

        # =============================================
        # DISPLAY WINDOWS
        # =============================================

        cv2.imshow(
            "1 RGB",
            draw_plate_region(frame_roi)
        )

        cv2.imshow(
            "2 Threshold",
            thresh
        )

        cv2.imshow(
            "3 All Contours",
            all_contours_img
        )

        cv2.imshow(
            "4 Area Filter",
            area_img
        )

        cv2.imshow(
            "5 Circularity Filter",
            circ_img
        )

        cv2.imshow(
            "6 Final Detection",
            final_img
        )

        # =============================================
        # KEYBOARD
        # =============================================

        key = cv2.waitKey(1) & 0xFF

        if key == ord('f'):

            paused = not paused

            if paused:
                print("Frame frozen")
            else:
                print("Live view resumed")

        elif key == ord('s'):

            cv2.imwrite(
                os.path.join(
                    SAVE_FOLDER,
                    "1_RGB.png"
                ),
                draw_plate_region(frame_roi)
            )

            cv2.imwrite(
                os.path.join(
                    SAVE_FOLDER,
                    "2_Threshold.png"
                ),
                thresh
            )

            cv2.imwrite(
                os.path.join(
                    SAVE_FOLDER,
                    "3_AllContours.png"
                ),
                all_contours_img
            )

            cv2.imwrite(
                os.path.join(
                    SAVE_FOLDER,
                    "4_AreaFilter.png"
                ),
                area_img
            )

            cv2.imwrite(
                os.path.join(
                    SAVE_FOLDER,
                    "5_CircularityFilter.png"
                ),
                circ_img
            )

            cv2.imwrite(
                os.path.join(
                    SAVE_FOLDER,
                    "6_FinalDetection.png"
                ),
                final_img
            )

            print("All report images saved.")

        elif key == 27:

            break

finally:

    pipeline.stop()

    cv2.destroyAllWindows()