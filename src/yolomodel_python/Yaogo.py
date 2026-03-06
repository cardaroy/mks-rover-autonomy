import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO
import os

MODEL_PATH = os.environ.get("CUBE_MODEL_PATH", "")
CONF_THRES = 0.6
DEPTH_MIN_M = 0.15
DEPTH_MAX_M = 5.0

if not MODEL_PATH:
    raise RuntimeError("CUBE_MODEL_PATH is empty. Set it to your YOLO model .pt file path.")
assert os.path.exists(MODEL_PATH), f"Model not found: {MODEL_PATH}"
print("Using model:", MODEL_PATH)

model = YOLO(MODEL_PATH)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)

align = rs.align(rs.stream.color)

# 取相机内参（color stream）
color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
intr = color_stream.get_intrinsics()
fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

print("Intrinsics:", fx, fy, cx, cy)
print("Depth scale:", depth_scale)

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_img = np.asanyarray(color_frame.get_data())
        depth_img = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale  # m

        results = model(color_img, conf=CONF_THRES, verbose=False)
        r0 = results[0]
        annotated = r0.plot()

        if r0.boxes is not None and len(r0.boxes) > 0:
            for box in r0.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0].cpu().numpy())

                # bbox clamp
                h, w = depth_img.shape
                x1 = max(0, min(x1, w-1)); x2 = max(0, min(x2, w-1))
                y1 = max(0, min(y1, h-1)); y2 = max(0, min(y2, h-1))
                if x2 <= x1 or y2 <= y1:
                    continue

                # center pixel (u,v)
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                # small window depth around center
                win = 9
                xs = max(0, u - win//2); xe = min(w, u + win//2 + 1)
                ys = max(0, v - win//2); ye = min(h, v + win//2 + 1)
                roi = depth_img[ys:ye, xs:xe]
                valid = roi[(roi > DEPTH_MIN_M) & (roi < DEPTH_MAX_M)]

                if valid.size == 0:
                    continue

                Z = float(np.median(valid))  # meters

                # pixel+depth -> camera 3D (meters)
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy

                # display
                txt = f"u,v=({u},{v}) Z={Z:.2f}m XYZ=({X:.2f},{Y:.2f},{Z:.2f}) conf={conf:.2f}"
                cv2.putText(annotated, txt, (x1, max(0, y1-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                # mark center
                cv2.circle(annotated, (u, v), 4, (0,255,0), -1)

                # also print to terminal (for logging)
                print(txt)

        cv2.imshow("RealSense YOLO + Depth + 3D", annotated)
        if cv2.waitKey(1) in [27, ord('q')]:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
