import cv2
import numpy as np
import os

SAVE_FOLDER = "report_images"
os.makedirs(SAVE_FOLDER, exist_ok=True)

MIN_AREA = 200
MAX_AREA = 2000
MIN_CIRCULARITY = 0.75

cap = cv2.VideoCapture(0)

while True:

    ret, frame = cap.read()

    if not ret:
        break

    # =====================================================
    # 1. GRAYSCALE
    # =====================================================

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # =====================================================
    # 2. BLUR
    # =====================================================

    blur = cv2.GaussianBlur(gray, (9, 9), 0)

    # =====================================================
    # 3. THRESHOLD
    # =====================================================

    _, binary = cv2.threshold(
        blur,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    # =====================================================
    # 4. ALL CONTOURS
    # =====================================================

    contours, _ = cv2.findContours(
        binary,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    contour_img = frame.copy()

    cv2.drawContours(
        contour_img,
        contours,
        -1,
        (0, 255, 0),
        2
    )

    # =====================================================
    # 5. AREA FILTER
    # =====================================================

    area_img = frame.copy()
    area_filtered = []

    for cnt in contours:

        area = cv2.contourArea(cnt)

        if MIN_AREA < area < MAX_AREA:
            area_filtered.append(cnt)

    cv2.drawContours(
        area_img,
        area_filtered,
        -1,
        (255, 0, 0),
        2
    )

    # =====================================================
    # 6. CIRCULARITY FILTER
    # =====================================================

    circ_img = frame.copy()
    circular_filtered = []

    for cnt in area_filtered:

        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)

        if perimeter == 0:
            continue

        circularity = (
            4 * np.pi * area /
            (perimeter * perimeter)
        )

        if circularity > MIN_CIRCULARITY:
            circular_filtered.append(cnt)

    cv2.drawContours(
        circ_img,
        circular_filtered,
        -1,
        (0, 0, 255),
        2
    )

    # =====================================================
    # 7. FINAL BALL DETECTION
    # =====================================================

    final_img = frame.copy()

    for cnt in circular_filtered:

        (x, y), radius = cv2.minEnclosingCircle(cnt)

        center = (int(x), int(y))
        radius = int(radius)

        cv2.circle(
            final_img,
            center,
            radius,
            (0, 255, 255),
            3
        )

        cv2.circle(
            final_img,
            center,
            5,
            (0, 0, 255),
            -1
        )

        cv2.putText(
            final_img,
            f"({center[0]}, {center[1]})",
            (center[0] + 10, center[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )

    # =====================================================
    # CONVERT GRAY IMAGES TO BGR
    # =====================================================

    gray_vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    blur_vis = cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)
    binary_vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

    # =====================================================
    # RESIZE ALL TO SAME SIZE
    # =====================================================

    W = 640
    H = 480

    def prep(img, title):

        img = cv2.resize(img, (W, H))

        cv2.putText(
            img,
            title,
            (15, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 255),
            2
        )

        return img

    original_panel = prep(frame.copy(), "Original")
    gray_panel = prep(gray_vis, "Grayscale")

    blur_panel = prep(blur_vis, "Blurred")
    binary_panel = prep(binary_vis, "Threshold")

    contour_panel = prep(contour_img, "All Contours")
    area_panel = prep(area_img, "Area Filter")

    circ_panel = prep(circ_img, "Circularity Filter")
    final_panel = prep(final_img, "Final Tracking")

    # =====================================================
    # BUILD DEBUG CANVAS
    # =====================================================

    row1 = np.hstack((original_panel, gray_panel))
    row2 = np.hstack((blur_panel, binary_panel))
    row3 = np.hstack((contour_panel, area_panel))
    row4 = np.hstack((circ_panel, final_panel))

    canvas = np.vstack((row1, row2, row3, row4))

    # =====================================================
    # SHOW CANVAS
    # =====================================================

    cv2.imshow("Ball Detection Pipeline", canvas)

    key = cv2.waitKey(1) & 0xFF

    # Save current pipeline figure
    if key == ord('s'):

        filename = os.path.join(
            SAVE_FOLDER,
            "ball_detection_pipeline.png"
        )

        cv2.imwrite(filename, canvas)

        print(f"Saved: {filename}")

    # Quit
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()