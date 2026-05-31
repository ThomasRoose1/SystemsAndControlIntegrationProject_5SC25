import cv2
import numpy as np
import os

# ==================================================
# PARAMETERS
# ==================================================

MIN_AREA = 150
MAX_AREA = 5000
MIN_CIRCULARITY = 0.75

SAVE_FOLDER = "processing_steps"

os.makedirs(SAVE_FOLDER, exist_ok=True)

# ==================================================
# LOAD IMAGE
# ==================================================

img = cv2.imread("test_image.jpg")

if img is None:
    raise Exception("Image not found")

# Save original image
cv2.imwrite(f"{SAVE_FOLDER}/1_original.png", img)

# ==================================================
# STEP 1: GRAYSCALE
# ==================================================

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

cv2.imwrite(f"{SAVE_FOLDER}/2_grayscale.png", gray)

# ==================================================
# STEP 2: BLUR
# ==================================================

blur = cv2.GaussianBlur(gray, (9,9), 0)

cv2.imwrite(f"{SAVE_FOLDER}/3_blurred.png", blur)

# ==================================================
# STEP 3: THRESHOLD
# ==================================================

_, binary = cv2.threshold(
    blur,
    0,
    255,
    cv2.THRESH_BINARY + cv2.THRESH_OTSU
)

cv2.imwrite(f"{SAVE_FOLDER}/4_binary.png", binary)

# ==================================================
# STEP 4: FIND CONTOURS
# ==================================================

contours, _ = cv2.findContours(
    binary,
    cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE
)

# Draw ALL contours
all_contours_img = img.copy()

cv2.drawContours(
    all_contours_img,
    contours,
    -1,
    (0,255,0),
    2
)

cv2.imwrite(
    f"{SAVE_FOLDER}/5_all_contours.png",
    all_contours_img
)

# ==================================================
# STEP 5: AREA FILTER
# ==================================================

area_filtered = []

area_img = img.copy()

for contour in contours:

    area = cv2.contourArea(contour)

    if MIN_AREA < area < MAX_AREA:
        area_filtered.append(contour)

cv2.drawContours(
    area_img,
    area_filtered,
    -1,
    (255,0,0),
    2
)

cv2.imwrite(
    f"{SAVE_FOLDER}/6_area_filtered.png",
    area_img
)

# ==================================================
# STEP 6: CIRCULARITY FILTER
# ==================================================

circular_filtered = []

circ_img = img.copy()

for contour in area_filtered:

    area = cv2.contourArea(contour)

    perimeter = cv2.arcLength(contour, True)

    if perimeter == 0:
        continue

    circularity = (
        4 * np.pi * area
    ) / (perimeter * perimeter)

    if circularity > MIN_CIRCULARITY:
        circular_filtered.append(contour)

cv2.drawContours(
    circ_img,
    circular_filtered,
    -1,
    (0,0,255),
    2
)

cv2.imwrite(
    f"{SAVE_FOLDER}/7_circularity_filtered.png",
    circ_img
)

# ==================================================
# STEP 7: FINAL BALL DETECTION
# ==================================================

final_img = img.copy()

for contour in circular_filtered:

    (x, y), radius = cv2.minEnclosingCircle(contour)

    center = (int(x), int(y))
    radius = int(radius)

    cv2.circle(
        final_img,
        center,
        radius,
        (0,255,255),
        3
    )

    cv2.circle(
        final_img,
        center,
        4,
        (0,0,255),
        -1
    )

cv2.imwrite(
    f"{SAVE_FOLDER}/8_final_detection.png",
    final_img
)

print("Processing images saved.")