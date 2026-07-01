import cv2
import numpy as np
import pyrealsense2 as rs
import math
import os
import json

# =====================================================
# CONFIGURATION
# =====================================================

WIDTH = 640
HEIGHT = 480
FPS = 60

PLATE_SIZE_MM = 390.0


THRESHOLD = 100
MASK_MARGIN = 15 # Margin to erode plate mask to avoid edge artifacts

MIN_RADIUS = 15
MAX_RADIUS = 30

MIN_AREA = 200
MAX_AREA = 2000

MIN_CIRCULARITY = 0.8

SAVE_FOLDER = r"C:\git\SystemsAndControlIntegrationProject_5SC25\3D Camera\Figures"

os.makedirs(SAVE_FOLDER, exist_ok=True)

# ================= LOAD CALIBRATION =================
CALIBRATION_FILE = 'plate_calibration.json'
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

ones = np.ones((4, 1))
plate_h = np.hstack([plate_quad, ones])
rotated_quad = (ROT_MAT @ plate_h.T).T.astype(np.int32)

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
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


def draw_plate_region(img):
    return img.copy()


# =====================================================
# REALSENSE
# =====================================================

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
pipeline.start(config)

# =====================================================
# WINDOWS
# =====================================================

WINDOW_W = 640
WINDOW_H = 480

cv2.namedWindow("1 RGB", cv2.WINDOW_NORMAL)
cv2.namedWindow("2 Gaussian Blur", cv2.WINDOW_NORMAL)
cv2.namedWindow("3 Threshold", cv2.WINDOW_NORMAL)
cv2.namedWindow("4 Candidate Filtering", cv2.WINDOW_NORMAL)
cv2.namedWindow("5 Final Detection", cv2.WINDOW_NORMAL)

cv2.resizeWindow("1 RGB", WINDOW_W, WINDOW_H)
cv2.resizeWindow("2 Gaussian Blur", WINDOW_W, WINDOW_H)
cv2.resizeWindow("3 Threshold", WINDOW_W, WINDOW_H)
cv2.resizeWindow("4 Candidate Filtering", WINDOW_W, WINDOW_H)
cv2.resizeWindow("5 Final Detection", WINDOW_W, WINDOW_H)

# Optional automatic layout
cv2.moveWindow("1 RGB", 0, 0)
cv2.moveWindow("2 Gaussian Blur", 650, 0)
cv2.moveWindow("3 Threshold", 1320, 0)
cv2.moveWindow("4 Candidate Filtering", 0, 520)
cv2.moveWindow("5 Final Detection", 650, 520)

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

            frame = np.asanyarray(color_frame.get_data())
            frozen_frame = frame.copy()

        else:
            frame = frozen_frame.copy()

        # =============================================
        # APPLY PLATE ROI IMMEDIATELY
        # =============================================
        frame = cv2.warpAffine(frame, ROT_MAT, (WIDTH, HEIGHT))
        frame = cv2.flip(frame, -1)

        frame_roi = frame.copy()

        # =============================================
        # GRAYSCALE
        # =============================================
        gray = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2GRAY)

        # =============================================
        # Gaussian Blur
        # =============================================
        blurred = cv2.GaussianBlur(gray, (5,5), 0)

        # =============================================
        # THRESHOLD
        # =============================================
        _, thresh = cv2.threshold(blurred, THRESHOLD, 255, cv2.THRESH_BINARY_INV)

        # =============================================
        # ALL CONTOURS
        # =============================================
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # =============================================
        # Filtering
        # =============================================
        filter_vis = cv2.cvtColor(mask.copy(), cv2.COLOR_GRAY2BGR)
        candidates = []

        for contour in contours:
            ctr = contour_centroid(contour)
            area = cv2.contourArea(contour)
            if area < MIN_AREA or area > MAX_AREA:
                cv2.drawContours(filter_vis, [contour], -1, (0,0,255), 2) # RED
                cv2.putText(filter_vis, f"A={area:.0f}", (ctr[0] + 5, ctr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
                continue

            circ = contour_circularity(contour)
            if circ < MIN_CIRCULARITY:
                cv2.drawContours(filter_vis, [contour], -1, (0,165,255), 2) # ORANGE
                cv2.putText(filter_vis, f"C={circ:.2f}", (ctr[0] + 5, ctr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
                continue

            # (x,y), radius = cv2.minEnclosingCircle(contour)
            # if radius < MIN_RADIUS or radius > MAX_RADIUS:
            #     cv2.drawContours(filter_vis, [contour], -1, (255,0,0), 2) # BLUE
            #     cv2.putText(filter_vis, f"R={radius:.0f}", (ctr[0] + 5, ctr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
            #     continue

            cv2.drawContours(filter_vis, [contour], -1, (0,255,0), 2) # GREEN
            candidates.append(contour)
        
        cv2.putText(filter_vis, "GREEN = Accepted", (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.putText(filter_vis, "RED = Area Reject", (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        cv2.putText(filter_vis, "ORANGE = Circularity Reject", (10,75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
        # cv2.putText(filter_vis, "BLUE = Radius Reject", (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
        
        # =============================================
        # FINAL DETECTION
        # =============================================
        final_img = draw_plate_region(frame_roi)

        if candidates:
            best_ball = max(candidates, key=cv2.contourArea)
            center = contour_centroid(best_ball)
            # =============================================
            # Convert image coordinates to plate coordinates
            # =============================================

            center_mm = cv2.perspectiveTransform(
                np.array([[[center[0], center[1]]]], dtype=np.float32),
                H
            )

            x_mm = float(center_mm[0,0,0])
            y_mm = -float(center_mm[0,0,1])      # Match controller coordinate system

            if center is not None:
                area = cv2.contourArea(best_ball)
                circ = contour_circularity(best_ball)

                (x,y), radius = cv2.minEnclosingCircle(best_ball)
                cv2.circle(final_img, (int(x), int(y)), int(radius), (255,0,255), 2)
                cv2.circle(final_img, center, 5, (0,0,255), -1)

                cv2.putText(final_img, f"Area = {area:.0f}", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.putText(final_img, f"Circularity = {circ:.2f}", (20,70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,255), 2)
                cv2.putText(final_img, f"X = {x_mm:+6.1f} mm", (20,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
                cv2.putText(final_img, f"Y = {y_mm:+6.1f} mm", (20,130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)

        # =============================================
        # DISPLAY WINDOWS
        # =============================================
        cv2.imshow("1 RGB", draw_plate_region(frame_roi))
        cv2.imshow("2 Gaussian Blur", blurred)
        cv2.imshow("3 Threshold", mask)
        cv2.imshow("4 Candidate Filtering", filter_vis)
        cv2.imshow("5 Final Detection", final_img)

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
            cv2.imwrite(os.path.join(SAVE_FOLDER, "1_RGB.png"), draw_plate_region(frame_roi))
            cv2.imwrite(os.path.join(SAVE_FOLDER, "2_Gaussian_Blur.png"), blurred)
            cv2.imwrite(os.path.join(SAVE_FOLDER, "3_Threshold.png"), mask)
            cv2.imwrite(os.path.join(SAVE_FOLDER, "4_Candidate_Filtering.png"), filter_vis)
            cv2.imwrite(os.path.join(SAVE_FOLDER, "5_FinalDetection.png"), final_img)

            print("All report images saved.")

        elif key == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()