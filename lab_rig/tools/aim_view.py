"""두 카메라 미리보기를 한 창에 나란히 — 카메라 위치 · 화각 맞출 때.

    source ~/jsh/env.sh && python3 ~/jsh/tools/aim_view.py

드라이버(GUI [센서 기동] 또는 런치)가 떠 있어야 한다. 미리보기 JPEG(image_rgb/compressed)를 그대로 보여준다.
키: g 격자 켜고 끄기 · c 십자선 켜고 끄기 · q / Esc 닫기
"""
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

CAMERAS = [("camera_bfs", "FLIR BFS"), ("lucid_tri054", "LUCID TRI054S")]
WINDOW = "aim view"


class Feed:
    def __init__(self):
        self.lock = threading.Lock()
        self.image = None
        self.stamps = []

    def push(self, msg):
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        now = time.time()
        with self.lock:
            self.image = img
            self.stamps = [t for t in self.stamps if now - t < 5.0] + [now]

    def get(self):
        with self.lock:
            fps = (len(self.stamps) - 1) / (self.stamps[-1] - self.stamps[0]) if len(self.stamps) > 1 else 0.0
            age = time.time() - self.stamps[-1] if self.stamps else None
            return self.image, fps, age


def overlay(img, label, fps, age, grid, cross):
    h, w = img.shape[:2]
    if grid:
        for k in (1, 2):
            cv2.line(img, (w * k // 3, 0), (w * k // 3, h), (90, 90, 90), 1, cv2.LINE_AA)
            cv2.line(img, (0, h * k // 3), (w, h * k // 3), (90, 90, 90), 1, cv2.LINE_AA)
    if cross:
        cx, cy, r = w // 2, h // 2, min(w, h) // 12
        cv2.line(img, (cx - 3 * r, cy), (cx + 3 * r, cy), (0, 220, 255), 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy - 3 * r), (cx, cy + 3 * r), (0, 220, 255), 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), r, (0, 220, 255), 1, cv2.LINE_AA)
    stale = age is None or age > 3.0
    text = f"{label}  {fps:.1f} fps" + ("  (no frames)" if stale else "")
    cv2.putText(img, text, (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255) if stale else (255, 255, 255), 2,
                cv2.LINE_AA)
    return img


def main():
    rclpy.init()
    node = rclpy.create_node("aim_view")
    feeds = {}
    for ns, _label in CAMERAS:
        feeds[ns] = Feed()
        node.create_subscription(CompressedImage, f"/{ns}/image_rgb/compressed", feeds[ns].push,
                                 qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 2400, 800)
    grid, cross, height = True, True, 760
    while True:
        tiles = []
        for ns, label in CAMERAS:
            img, fps, age = feeds[ns].get()
            if img is None:
                tile = np.zeros((height, int(height * 1.55), 3), np.uint8)
            else:
                tile = cv2.resize(img, (int(img.shape[1] * height / img.shape[0]), height),
                                  interpolation=cv2.INTER_AREA)
            tiles.append(overlay(tile, label, fps, age, grid, cross))
        sep = np.full((height, 6, 3), 40, np.uint8)
        cv2.imshow(WINDOW, np.hstack([tiles[0], sep, tiles[1]]))
        key = cv2.waitKey(50) & 0xFF
        if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key == ord("g"):
            grid = not grid
        if key == ord("c"):
            cross = not cross
    cv2.destroyAllWindows()
    executor.shutdown()
    spinner.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
