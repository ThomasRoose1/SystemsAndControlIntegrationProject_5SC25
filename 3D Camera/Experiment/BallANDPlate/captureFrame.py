import cv2

# Open the default camera (0)
cap = cv2.VideoCapture(0)

# Check if the camera opened correctly
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Read one frame
ret, frame = cap.read()

if ret:
    # Save the captured frame
    cv2.imwrite("imageFrame.png", frame)
    print("Image saved as imageFrame.png")
else:
    print("Failed to capture image")

# Release the camera
cap.release()
cv2.destroyAllWindows()
