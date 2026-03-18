import argparse
import logging
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from pupil_apriltags import Detector


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

DEFAULT_GST = (
    "rtspsrc location=rtsp://100.103.8.66:8554/video latency=0 ! "
    "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
    "appsink drop=true max-buffers=1 sync=false"
)
BASE_DIR = Path(__file__).resolve().parent
CALIB_PATH = BASE_DIR / "calibration_data" / "camera_calibration.npz"
TAG_SIZE = 0.09
PPM = 400
X_MIN, X_MAX = -0.4, 0.4
Y_MIN, Y_MAX = -0.4, 0.4
WINDOW_NAME = "AprilTag BEV"
ROI_MARGIN = 80
MAX_STALE_FRAMES = 3


def parse_args():
    parser = argparse.ArgumentParser(description="Low-latency AprilTag to BEV")
    parser.add_argument("--image", help="单张图片路径；不传则默认读取 RTSP")
    parser.add_argument("--gst", default=DEFAULT_GST, help="GStreamer pipeline")
    parser.add_argument(
        "--calibration",
        default=str(CALIB_PATH),
        help="相机标定 npz 文件路径",
    )
    parser.add_argument("--target-id", type=int, default=0, help="只使用指定 tag_id")
    parser.add_argument("--quad-decimate", type=float, default=1.5, help="检测降采样倍率")
    parser.add_argument("--threads", type=int, default=min(4, os.cpu_count() or 2), help="检测线程数")
    return parser.parse_args()


class LatestFrameGrabber:
    def __init__(self, gst):
        self.cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open GStreamer stream")
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.frame = None
        self.frame_id = 0
        self.read_fps = 0.0
        self._read_count = 0
        self._last_stat_t = time.perf_counter()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        deadline = time.perf_counter() + 3.0
        while self.frame is None and time.perf_counter() < deadline:
            time.sleep(0.01)
        if self.frame is None:
            raise RuntimeError("Cannot read first frame from RTSP")
        return self.frame.copy()

    def _run(self):
        while not self.stop_event.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self.lock:
                self.frame = frame
                self.frame_id += 1
            self._read_count += 1
            now = time.perf_counter()
            if now - self._last_stat_t >= 1.0:
                self.read_fps = self._read_count / (now - self._last_stat_t)
                self._read_count = 0
                self._last_stat_t = now

    def get_latest(self, last_seen_id):
        with self.lock:
            if self.frame is None or self.frame_id == last_seen_id:
                return last_seen_id, None
            return self.frame_id, self.frame.copy()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.cap.release()


def build_detector(args):
    return Detector(
        families="tag36h11",
        nthreads=max(1, args.threads),
        quad_decimate=args.quad_decimate,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.5,
        debug=0,
    )


def build_bev_constants():
    obj = np.array(
        [
            [TAG_SIZE / 2, TAG_SIZE / 2],
            [-TAG_SIZE / 2, TAG_SIZE / 2],
            [-TAG_SIZE / 2, -TAG_SIZE / 2],
            [TAG_SIZE / 2, -TAG_SIZE / 2],
        ],
        dtype=np.float32,
    )
    world_to_bev = np.array(
        [
            [1 / PPM, 0, X_MIN],
            [0, -1 / PPM, Y_MAX],
            [0, 0, 1],
        ],
        dtype=np.float32,
    )
    bev_w = int((X_MAX - X_MIN) * PPM)
    bev_h = int((Y_MAX - Y_MIN) * PPM)
    return obj, world_to_bev, bev_w, bev_h


def load_maps(frame_shape, calibration_path):
    data = np.load(calibration_path)
    camera_matrix = data["camera_matrix"].astype(np.float32)
    dist_coeffs = data["dist_coeffs"].astype(np.float32)
    h, w = frame_shape[:2]
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 0, (w, h))
    return cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_32FC1
    )


def pick_target(tags, target_id):
    for det in tags:
        if det.tag_id == target_id:
            return det
    return None


def shift_detection(det, offset_x, offset_y):
    if det is None:
        return None
    det.corners = det.corners + np.array([offset_x, offset_y], dtype=np.float32)
    det.center = det.center + np.array([offset_x, offset_y], dtype=np.float32)
    return det


def detect_tag(detector, gray, target_id, last_corners):
    if last_corners is not None:
        xmin = max(0, int(np.floor(np.min(last_corners[:, 0]))) - ROI_MARGIN)
        xmax = min(gray.shape[1], int(np.ceil(np.max(last_corners[:, 0]))) + ROI_MARGIN)
        ymin = max(0, int(np.floor(np.min(last_corners[:, 1]))) - ROI_MARGIN)
        ymax = min(gray.shape[0], int(np.ceil(np.max(last_corners[:, 1]))) + ROI_MARGIN)
        if xmax - xmin > 20 and ymax - ymin > 20:
            roi = gray[ymin:ymax, xmin:xmax]
            det = pick_target(detector.detect(roi), target_id)
            if det is not None:
                return shift_detection(det, xmin, ymin), "roi"

    det = pick_target(detector.detect(gray), target_id)
    if det is not None:
        return det, "full"
    return None, "miss"


def render_bev(frame, corners, obj_points, world_to_bev, bev_w, bev_h):
    homography = cv2.getPerspectiveTransform(obj_points, corners)
    matrix = homography @ world_to_bev
    return cv2.warpPerspective(
        frame,
        matrix,
        (bev_w, bev_h),
        flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
    )


def resize_to_height(image, target_h):
    if image.shape[0] == target_h:
        return image
    scale = target_h / image.shape[0]
    target_w = max(1, int(round(image.shape[1] * scale)))
    return cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_LINEAR)


def compose_view(vis, bev):
    bev_resized = resize_to_height(bev, vis.shape[0])
    canvas = np.zeros((vis.shape[0], vis.shape[1] + bev_resized.shape[1], 3), dtype=np.uint8)
    canvas[:, : vis.shape[1]] = vis
    canvas[:, vis.shape[1] :] = bev_resized
    return canvas


def draw_overlay(vis, det, stale_count, mode, detect_ms, proc_fps, read_fps):
    if det is not None:
        pts = det.corners.astype(int)
        cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
        for i, point in enumerate(pts):
            cv2.circle(vis, tuple(point), 4, (0, 0, 255), -1)
            cv2.putText(vis, str(i), tuple(point), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        state = "TRACK" if stale_count > 0 else "LIVE"
        family = det.tag_family.decode() if isinstance(det.tag_family, bytes) else det.tag_family
        cv2.putText(vis, f"{family}:{det.tag_id} {state} {mode}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    else:
        cv2.putText(vis, "tag lost", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

    cv2.putText(
        vis,
        f"detect {detect_ms:.1f} ms | proc {proc_fps:.1f} fps | read {read_fps:.1f} fps",
        (20, 75),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 0),
        2,
    )


def process_frame(frame, map1, map2, detector, target_id, obj_points, world_to_bev, bev_w, bev_h, last_corners):
    undistort = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
    gray = cv2.cvtColor(undistort, cv2.COLOR_BGR2GRAY)
    t0 = time.perf_counter()
    det, mode = detect_tag(detector, gray, target_id, last_corners)
    detect_ms = (time.perf_counter() - t0) * 1000.0
    if det is None:
        return undistort, np.zeros((bev_h, bev_w, 3), dtype=np.uint8), None, mode, detect_ms
    corners = det.corners.astype(np.float32)
    bev = render_bev(undistort, corners, obj_points, world_to_bev, bev_w, bev_h)
    return undistort, bev, det, mode, detect_ms


def run_image_mode(args):
    frame = cv2.imread(args.image)
    if frame is None:
        raise RuntimeError(f"Cannot read image: {args.image}")
    map1, map2 = load_maps(frame.shape, args.calibration)
    detector = build_detector(args)
    obj_points, world_to_bev, bev_w, bev_h = build_bev_constants()
    vis, bev, det, mode, detect_ms = process_frame(
        frame, map1, map2, detector, args.target_id, obj_points, world_to_bev, bev_w, bev_h, None
    )
    draw_overlay(vis, det, 0, mode, detect_ms, 0.0, 0.0)
    combined = compose_view(vis, bev)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1600, 900)
    cv2.imshow(WINDOW_NAME, combined)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_stream_mode(args):
    grabber = LatestFrameGrabber(args.gst)
    first_frame = grabber.start()
    map1, map2 = load_maps(first_frame.shape, args.calibration)
    detector = build_detector(args)
    obj_points, world_to_bev, bev_w, bev_h = build_bev_constants()

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1600, 900)

    last_frame_id = 0
    last_corners = None
    last_det = None
    stale_count = MAX_STALE_FRAMES + 1
    proc_count = 0
    proc_fps = 0.0
    last_stat_t = time.perf_counter()

    try:
        while True:
            last_frame_id, frame = grabber.get_latest(last_frame_id)
            if frame is None:
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                time.sleep(0.001)
                continue

            vis, bev, det, mode, detect_ms = process_frame(
                frame, map1, map2, detector, args.target_id, obj_points, world_to_bev, bev_w, bev_h, last_corners
            )

            if det is not None:
                last_det = det
                last_corners = det.corners.astype(np.float32)
                stale_count = 0
            elif last_det is not None and stale_count < MAX_STALE_FRAMES:
                stale_count += 1
                det = last_det
                bev = render_bev(vis, det.corners.astype(np.float32), obj_points, world_to_bev, bev_w, bev_h)
            else:
                stale_count = MAX_STALE_FRAMES + 1
                last_det = None
                last_corners = None

            proc_count += 1
            now = time.perf_counter()
            if now - last_stat_t >= 1.0:
                proc_fps = proc_count / (now - last_stat_t)
                proc_count = 0
                last_stat_t = now
                logger.info(
                    "detect=%.1fms proc=%.1ffps read=%.1ffps mode=%s stale=%d",
                    detect_ms,
                    proc_fps,
                    grabber.read_fps,
                    mode,
                    stale_count,
                )

            draw_overlay(vis, det, stale_count, mode, detect_ms, proc_fps, grabber.read_fps)
            cv2.imshow(WINDOW_NAME, compose_view(vis, bev))

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        grabber.stop()
        cv2.destroyAllWindows()


def main():
    args = parse_args()
    if args.image:
        run_image_mode(args)
    else:
        run_stream_mode(args)


if __name__ == "__main__":
    main()
