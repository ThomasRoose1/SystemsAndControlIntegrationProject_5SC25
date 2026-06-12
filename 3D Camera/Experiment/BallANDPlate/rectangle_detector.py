import cv2
import numpy as np

# Load image
image = cv2.imread('imageFrame.png')
orig = image.copy()

# Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Create a mask for floodFill (2 pixels larger than the image)
h, w = gray.shape
mask = np.zeros((h+2, w+2), np.uint8)

# Seed point (adjusted for center of plate)
seed_point = (w // 2 - 50, h // 2 - 50)

# Flood fill parameters
fill_val = 255
lo_diff = 1
up_diff = 1

# Flood fill
flooded = gray.copy()
cv2.floodFill(flooded, mask, seedPoint=seed_point, newVal=fill_val,
              loDiff=lo_diff, upDiff=up_diff)

# Threshold
_, thresh = cv2.threshold(flooded, 254, 255, cv2.THRESH_BINARY)

# Find contours
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = sorted(contours, key=cv2.contourArea, reverse=True)

# Proceed with the largest contour
for cnt in contours:
    if cv2.contourArea(cnt) > 100:
        # Fit min area rect
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.intp(box)

        # Draw the rectangle
        cv2.drawContours(orig, [box], 0, (0, 255, 0), 2)

        # Sort box points by Y (bottom two will have highest Y values)
        box_sorted = sorted(box, key=lambda p: p[1], reverse=True)
        bottom_line = box_sorted[:2]

        # Sort left-to-right
        bottom_line = sorted(bottom_line, key=lambda p: p[0])

        pt1, pt2 = bottom_line
        print(f"Bottom line start: {pt1}, end: {pt2}")

        # Optionally draw the bottom line in red
        cv2.line(orig, tuple(pt1), tuple(pt2), (0, 0, 255), 2)

        break

# Show result
cv2.imshow("Flood-Filled Mask", thresh)
cv2.imshow("Detected Rectangle + Bottom Edge", orig)
# Calculate the angle in degrees
dx = pt2[0] - pt1[0]
dy = pt2[1] - pt1[1]
angle_rad = np.arctan2(dy, dx)
angle_deg = np.degrees(angle_rad)

# Normalize angle to range [-90, 90] for display
if angle_deg > 90:
    angle_deg -= 180
elif angle_deg < -90:
    angle_deg += 180

print(f"Angle of bottom line with horizontal: {angle_deg:.2f} degrees")

(center_x, center_y), (width, height), angle = rect
print(f"Rectangle width: {width:.2f} px, height: {height:.2f} px")
print(f"Rectangle center_x: {center_x:.2f} px, center_y: {center_y:.2f} px")
conversion_factor = height/0.391
print(f"Conversion factor: {conversion_factor:.2f}")
cv2.waitKey(0)
cv2.destroyAllWindows()
