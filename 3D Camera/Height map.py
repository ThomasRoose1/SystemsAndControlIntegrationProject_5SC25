
# Height Map.py
import cv2, numpy as np, pyrealsense2 as rs, json
WIDTH,HEIGHT=640,480
FPS_COLOR,FPS_DEPTH=60,90
ANGLE=0
CENTER=(WIDTH//2,HEIGHT//2)
ROT_MAT=cv2.getRotationMatrix2D(CENTER,ANGLE,1.0)
CALIB_FILE="plate_calibration.json"
DEPTH_FRAMES_TO_AVERAGE=100
MASK_MARGIN=10
HEIGHT_RANGE_MM=3.0
mouse_x=mouse_y=0
def mouse(event,x,y,flags,param):
    global mouse_x,mouse_y
    mouse_x,mouse_y=x,y
with open(CALIB_FILE) as f: calib=json.load(f)
plate_quad=np.array(calib["plate_quad"],dtype=np.int32)
plate_mask=np.zeros((HEIGHT,WIDTH),np.uint8)
cv2.fillPoly(plate_mask,[plate_quad],255)
plate_mask=cv2.erode(plate_mask,np.ones((2*MASK_MARGIN+1,2*MASK_MARGIN+1),np.uint8),1)
pipeline=rs.pipeline();config=rs.config()
config.enable_stream(rs.stream.color,WIDTH,HEIGHT,rs.format.bgr8,FPS_COLOR)
config.enable_stream(rs.stream.depth,WIDTH,HEIGHT,rs.format.z16,FPS_DEPTH)
profile=pipeline.start(config)
align=rs.align(rs.stream.color)
depth_scale=profile.get_device().first_depth_sensor().get_depth_scale()
temporal=rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha,0.4)
temporal.set_option(rs.option.filter_smooth_delta,20)
cv2.namedWindow("Height Map");cv2.setMouseCallback("Height Map",mouse)
def capture():
    s=None
    for _ in range(DEPTH_FRAMES_TO_AVERAGE):
        fr=align.process(pipeline.wait_for_frames())
        d=fr.get_depth_frame()
        if not d: continue
        d=temporal.process(d)
        a=np.asanyarray(d.get_data()).astype(np.float32)*depth_scale*1000
        a=cv2.flip(cv2.warpAffine(a,ROT_MAT,(WIDTH,HEIGHT)),-1)
        s=a if s is None else s+a
    return s/DEPTH_FRAMES_TO_AVERAGE
heat=None;rel=None
try:
    while True:
        img=np.zeros((HEIGHT,WIDTH,3),np.uint8) if heat is None else heat.copy()
        if heat is None:
            cv2.putText(img,"SPACE=capture S=save ESC=quit",(20,40),0,0.7,(255,255,255),2)
        else:
            if 0<=mouse_x<WIDTH and 0<=mouse_y<HEIGHT and plate_mask[mouse_y,mouse_x]:
                cv2.circle(img,(mouse_x,mouse_y),3,(255,255,255),-1)
                cv2.putText(img,f"{rel[mouse_y,mouse_x]:+.2f} mm",(mouse_x+10,mouse_y-10),0,0.6,(255,255,255),2)
        cv2.imshow("Height Map",img)
        k=cv2.waitKey(1)&0xFF
        if k==27: break
        if k==32:
            depth=capture()
            ys, xs = np.where(plate_mask > 0)

            zs = depth[ys, xs]

            A = np.c_[xs, ys, np.ones_like(xs)]

            coef, _, _, _ = np.linalg.lstsq(
                A,
                zs,
                rcond=None
            )

            plane = (
                coef[0] * np.arange(WIDTH)[None, :]
                + coef[1] * np.arange(HEIGHT)[:, None]
                + coef[2]
            )

            rel = plane - depth
            rel[plate_mask==0]=np.nan
            clip=np.clip(rel,-HEIGHT_RANGE_MM,HEIGHT_RANGE_MM)
            norm=cv2.normalize(np.nan_to_num(clip),None,0,255,cv2.NORM_MINMAX).astype(np.uint8)
            heat=cv2.applyColorMap(norm,cv2.COLORMAP_TURBO)
            max_index = np.nanargmax(rel)

            max_y, max_x = np.unravel_index(
                max_index,
                rel.shape
            )
            cv2.drawMarker(
                heat,
                (max_x, max_y),
                (255,255,255),
                cv2.MARKER_CROSS,
                20,
                2
            )
            cv2.putText(
                heat,
                f"{np.nanmax(rel):.2f} mm",
                (max_x+10, max_y-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255,255,255),
                2
            )
            heat[plate_mask==0]=(30,30,30)
            cv2.putText(
                heat,
                f"Max : {np.nanmax(rel):5.2f} mm",
                (20,30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255,255,255),
                2
            )

            cv2.putText(
                heat,
                f"Min : {np.nanmin(rel):5.2f} mm",
                (20,55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255,255,255),
                2
            )

            cv2.putText(
                heat,
                f"Std : {np.nanstd(rel):5.2f} mm",
                (20,80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255,255,255),
                2
            )
            cv2.polylines(heat,[plate_quad],True,(255,255,255),2)
        if k==ord('s') and heat is not None:
            np.save("height_map.npy",rel);cv2.imwrite("height_map.png",heat)
finally:
    pipeline.stop();cv2.destroyAllWindows()
