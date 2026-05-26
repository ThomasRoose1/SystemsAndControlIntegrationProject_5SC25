import cv2
import numpy as np
import pyrealsense2 as rs
import json

# ================= CONFIG =================
WIDTH = 640
HEIGHT = 480
FPS = 60

ANGLE = -0.65
CENTER = (WIDTH // 2, HEIGHT // 2)
ROT_MAT = cv2.getRotationMatrix2D(CENTER, ANGLE, 1.0)

OUTPUT_FILE = 'plate_calibration.json'

clicked_points = []
current_frame = None

# ================= MOUSE CALLBACK =================
def mouse_callback(event, x, y, flags, param):
    global clicked_points

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(clicked_points) < 4:
            clicked_points.append((x, y))
            print(f'Point {len(clicked_points)}: ({x}, {y})')


def draw_overlay(frame):
    display = frame.copy()

    labels = ['TL', 'TR', 'BR', 'BL']

    for i, pt in enumerate(clicked_points):
        cv2.circle(display, pt, 6, (0, 0, 255), -1)
        cv2.putText(display,
                    f'{i+1}:{labels[i]}',
                    (pt[0] + 10, pt[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2)

    if len(clicked_points) >= 2:
        for i in range(len(clicked_points) - 1):
            cv2.line(display, clicked_points[i], clicked_points[i+1], (255, 0, 255), 2)

    if len(clicked_points) == 4:
        cv2.line(display, clicked_points[3], clicked_points[0], (255, 0, 255), 2)

        xs = [p[0] for p in clicked_points]
        ys = [p[1] for p in clicked_points]

        x1 = min(xs)
        x2 = max(xs)
        y1 = min(ys)
        y2 = max(ys)

        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)

    instructions = [
        'Click plate corners in order:',
        '1 TL   2 TR   3 BR   4 BL',
        'ENTER = save   R = reset   ESC = quit'
    ]

    y_text = 30
    for line in instructions:
        cv2.putText(display,
                    line,
                    (20, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2)
        y_text += 30

    return display


# ================= CAMERA INIT =================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
pipeline.start(config)

cv2.namedWindow('Plate Calibration')
cv2.setMouseCallback('Plate Calibration', mouse_callback)

# ================= MAIN LOOP =================
try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        frame = cv2.warpAffine(frame, ROT_MAT, (WIDTH, HEIGHT))
        frame = cv2.flip(frame, -1)

        current_frame = frame.copy()
        display = draw_overlay(current_frame)

        cv2.imshow('Plate Calibration', display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            clicked_points = []
            print('Calibration points reset.')

        elif key == 13:
            if len(clicked_points) == 4:
                xs = [p[0] for p in clicked_points]
                ys = [p[1] for p in clicked_points]

                calibration_data = {
                    'plate_size_mm': 400,
                    'top_left': clicked_points[0],
                    'top_right': clicked_points[1],
                    'bottom_right': clicked_points[2],
                    'bottom_left': clicked_points[3],
                    'plate_quad': [
                        clicked_points[0],
                        clicked_points[1],
                        clicked_points[2],
                        clicked_points[3]
                    ],
                    'plate_x1': min(xs),
                    'plate_x2': max(xs),
                    'plate_y1': min(ys),
                    'plate_y2': max(ys)
                }

                with open(OUTPUT_FILE, 'w') as f:
                    json.dump(calibration_data, f, indent=4)

                print('\nCalibration saved to', OUTPUT_FILE)
                print(json.dumps(calibration_data, indent=4))
                break
            else:
                print('Need exactly 4 points before saving.')

        elif key == 27:
            print('Calibration cancelled.')
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
