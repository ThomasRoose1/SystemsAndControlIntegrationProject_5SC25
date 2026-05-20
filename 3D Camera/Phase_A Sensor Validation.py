import pyrealsense2 as rs
import cv2
import numpy as np
import time

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)

pipeline.start(config)

t0 = time.time()
count = 0

while True:
    frames = pipeline.wait_for_frames()
    color = frames.get_color_frame()
    frame = np.asanyarray(color.get_data())

    count += 1
    fps = count / (time.time() - t0)

    cv2.putText(frame, f"FPS: {fps:.1f}", (20,40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    cv2.imshow("D435 RGB", frame)

    if cv2.waitKey(1) == 27:
        break

pipeline.stop()
cv2.destroyAllWindows()