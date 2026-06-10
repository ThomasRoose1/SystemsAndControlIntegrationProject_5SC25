import socket
import cv2
import numpy as np
import os
import time

# Define server address and port
serverAddressPort = ("192.168.140.8", 49001)  # IP address and port of the MicrolabBox where the ball position data is sent
bufferSize = 32

# Create UDP socket
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# UDPClientSocket.bind(("192.168.140.100", 0))



#video = cv2.VideoCapture('video_test.mp4')  # Load video from file or use cv2.VideoCapture(0) for live camera feed
video = cv2.VideoCapture(0)
width = int(video.get(3))  # Frame width
height = int(video.get(4))  # Frame height
fps = video.get(cv2.CAP_PROP_FPS)  # Get FPS of the video
size = (width, height)
print(width)
print(height)
# Parameters
angle = -0.65  # Counter-clockwise rotation angle
center = (width // 2, height // 2)  # Center of the frame
scale = 1.0  # No scaling
M = cv2.getRotationMatrix2D(center, angle, scale)  # Rotation matrix

#algorithm parameters
threshold_value = 50 #if the center pixel value is below 50, it implies that the algorithm thinks that the ball is under the provided pixel. In detect_ball_custom.
min_width = 20 #algorithm looks for balls which are bigger than this value in pixels.
max_width = 60 #algorithm looks for balls wich are smaller than this value in pixels.


# Threshold for pixel intensity to decide if flood fill is needed
threshold_value = 100

# Define ball detection methods

def detect_ball_houghcircles(frame):
    """
    Use HoughCircles to detect the approximate position of the ball.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (17, 17), 0)
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1.6, 300, param1=100, param2=40, minRadius=10, maxRadius=200)
    return circles

def detect_ball_custom(frame, prev_circle):
    """
    Custom method for more accurate ball tracking using floodfill if necessary.
    """
    if prev_circle is None:
        return None  # If no previous circle, can't track yet

    x, y = prev_circle[0], prev_circle[1]
    if 0 > x:
        x = 0
    elif frame.shape[1] < x:
        x =  frame.shape[1]
    else:
        x=x
    if 0 > y:
        y = 0
    elif frame.shape[0] < y:
        y = frame.shape[0]
    else:
        y = y
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Check if the pixel intensity at the center of the detected ball is below a threshold
    pixel_val = frame[y, x]
    #print(f"Pixel value at center: {pixel_val}")

    if pixel_val < threshold_value:  # If intensity is below the threshold, apply flood fill
        flooded = frame.copy()
        mask = np.zeros((flooded.shape[0] + 2, flooded.shape[1] + 2), np.uint8)

        # Flood fill starting from the detected position
        seed_point = (x, y)
        fill_val = 255  # Fill the ball with black, as the ball is black (0 intensity in grayscale)
        lo_diff = 2  # Low difference for flood fill
        up_diff = 2  # Upper difference for flood fill
        
        # Apply flood fill
        cv2.floodFill(flooded, mask, seedPoint=seed_point, newVal=fill_val, loDiff=lo_diff, upDiff=up_diff)

        # Show the flooded image
        #cv2.imshow("Flood Filled Image", flooded)
        #cv2.waitKey(0)

        # Threshold the image to detect the black ball (background is white)
        _, thresh = cv2.threshold(flooded, 254, 255, cv2.THRESH_BINARY)  # Inverse threshold for black ball on white background
        #cv2.imshow("Thresholded Image", thresh)  # Show the thresholded image
        #cv2.waitKey(0)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        # If we have any contours, use the largest one to track the ball
        if contours:
            ball_contour = contours[0]
            (x, y, w, h) = cv2.boundingRect(ball_contour)  # Get bounding rectangle of the ball
            
            # Draw the bounding rectangle on the flooded image
            cv2.rectangle(flooded, (x, y), (x + w, y + h), 255, 2)  # White rectangle on black background
            cv2.circle(flooded, (x + w // 2, y + h // 2), 5, 255, -1)  # Draw a center point
            #save_cropped_ball_image(flooded, (x,y,w), output_dir)
            if min_width < w < max_width:
                return (x + w // 2, y + h // 2, w, h)  # Return the center of the largest contour and its width and height
        
    return None  # Return None if custom detection fails

def track_ball(frame, prev_circle=None, use_hough=True):
    """
    Track the ball position using either HoughCircles or custom detection.
    Switch to custom detection once ball is found, and revert to HoughCircles if lost.
    """
    if use_hough:
        # Use HoughCircles for initial detection
        circles = detect_ball_houghcircles(frame)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            # Select the closest circle to the previous position (if available)
            chosen = circles[0, 0]
            for j in circles[0, :]:
                if prev_circle is not None and dist(chosen[0], chosen[1], prev_circle[0], prev_circle[1]) > dist(j[0], j[1], prev_circle[0], prev_circle[1]):
                    chosen = j
            #save_cropped_ball_image(frame, chosen, output_dir)
            return chosen, False  # Found using HoughCircles, continue to custom tracking

        else:
            # Display 'No ball found' message when using HoughCircles
            cv2.putText(frame, 'No ball found', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            return None, True  # Ball not found, continue using HoughCircles

    else:
        # Use custom tracking if HoughCircles failed
        chosen = detect_ball_custom(frame, prev_circle)
        if chosen is not None:
            x, y, w, h = chosen
            # Draw a rectangle around the detected ball
            cv2.circle(frame, (chosen[0], chosen[1]), 1, (0, 100, 100), 3)  # Draw the center of the circle
            cv2.circle(frame, (chosen[0], chosen[1]), int(chosen[2]/2), (255, 0, 255), 3)  # Draw the circle perimeter
            return chosen, False  # Ball found using custom method
        else:
            return None, True  # Ball lost, switch back to HoughCircles


def dist(x1, y1, x2, y2):
    """
    Calculate Euclidean distance between two points.
    """
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5


def send_ball_position_to_microlab(chosen):
    """
    Send the ball position as bytes to the MicrolabBox.
    """
    x = int(chosen[0])
    x_bytes = x.to_bytes(2, byteorder='little')
    x0, x1 = x_bytes[0], x_bytes[1]
    y = int(chosen[1])
    y_bytes = y.to_bytes(2, byteorder='little')
    y0, y1 = y_bytes[0], y_bytes[1]

    # Debug print to terminal
    print(f"Sending ball position -> X: {x}, Y: {y}") # Added PdT

    # Prepare byte array with ball position data
    bytesToSend = bytearray([y0, y1, 0, 0, x0, x1])

    # Send ball position to the MicrolabBox
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)


def save_cropped_ball_image(frame, circle, output_dir):
    """

    Crop the ball from the frame based on the circle's position and save it as an image.
    """
    print(frame.shape[1])
    print(frame.shape[0])
    x, y, radius = circle
    radius = radius +20
    if radius > x:
        crop_x1 = 0
    else:
        crop_x1 = x - radius
    if radius > y:
        crop_y1 = 0
    else:
        crop_y1 = y - radius
    if frame.shape[1] < x + radius:
        crop_x2 =  frame.shape[1]
    else:
        crop_x2 = x + radius
    if frame.shape[0] < y + radius:
        crop_y2 = frame.shape[0]
    else:
        crop_y2 = y + radius
    # Ensure that the crop region is valid
    if crop_x1 < crop_x2 and crop_y1 < crop_y2:
        cropped_ball = frame[crop_y1:crop_y2, crop_x1:crop_x2]

        # Check if the cropped ball image is not empty
        if cropped_ball.size > 0:
            # Save the cropped image for debugging purposes
            image_filename = os.path.join(output_dir, f"ball_{int(time.time())}.png")
            cv2.imwrite(image_filename, cropped_ball)
            print(f"Saved cropped ball image: {image_filename}")
        else:
            print("Error: Cropped image is empty.")
    else:
        print("Error: Invalid crop region.")
import time  # Import time module

# Main loop
prev_circle = None  # Previous ball position (x, y, radius)
ref_circle = None   # Reference circle for extrapolated position
dx = None           # X velocity
dy = None           # Y velocity
use_hough = True    # Start with HoughCircles for initial ball detection


while True:
    ret, frame = video.read()

    # Break the loop if the video has reached the end
    if not ret or frame is None:
        print('End of video or failed to retrieve frame.')
        break  # Exit the loop if no frame is retrieved or end of video is reached
    
    # Capture start time before processing the frame
    start_time = time.time()

    # Apply rotation to frame (if necessary)
    frame = cv2.warpAffine(frame, M, (width, height))

    # Track ball position
    if prev_circle is not None and dx is not None and dy is not None:
        # Extrapolate the ball's expected position based on previous velocity (dx, dy)
        ref_circle = (int(prev_circle[0] + dx), int(prev_circle[1] + dy))
    else:
        ref_circle = prev_circle  # Use HoughCircles for initial detection if no previous data is available

    chosen, use_hough = track_ball(frame, ref_circle, use_hough)
    if chosen is not None:
        # Update previous circle position and calculate velocities
        if prev_circle is not None:
            dx = int(chosen[0]) - int(prev_circle[0])  # X velocity
            dy = int(chosen[1]) - int(prev_circle[1])  # Y velocity
        # Update the previous circle to the current one
        prev_circle = chosen
        
        # Send ball position to the MicrolabBox if using custom tracking
        if use_hough is False:
            send_ball_position_to_microlab(chosen)

    else:
        # Ball position lost, reset everything
        dx = None
        dy = None
        prev_circle = None
        ref_circle = None

    # Capture end time after sending the position
    end_time = time.time()

    # Calculate the time taken to process the frame
    elapsed_time = end_time - start_time
    # Display the processed frame
    cv2.imshow('test_circles', frame)

    cv2.waitKey(5)  # Control playback speed to match desired FPS

cv2.destroyAllWindows()