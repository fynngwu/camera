"""Distortion correction demo - simplified."""

import cv2
import numpy as np
from pathlib import Path
import argparse


def create_pipeline(url: str) -> str:
    """Create GStreamer pipeline for RTSP stream."""
    return f"rtspsrc location={url} latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"


def load_calibration(path: str):
    """Load camera calibration data."""
    p = Path(path)
    if not p.exists():
        print(f"Error: Calibration file not found: {path}")
        return None
    data = np.load(p)
    return data


def create_undistort_maps(K, dist, img_size, alpha=0):
    """Create undistort maps using remap for efficiency."""
    w, h = img_size
    newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha, (w, h))
    map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (w, h), cv2.CV_16SC2)
    return map1, map2, newK, roi


def apply_undistort(img, map1, map2):
    """Apply undistort using precomputed maps."""
    return cv2.remap(img, map1, map2, cv2.INTER_LINEAR)


def main():
    parser = argparse.ArgumentParser(description="Undistort demo")
    parser.add_argument("--camera", type=int, default=0, help="Camera ID")
    parser.add_argument("--calibration", type=str, default="calibration_data/camera_calibration.npz")
    parser.add_argument("--mode", choices=["live", "images", "rtsp"], default="rtsp")
    parser.add_argument("--images-dir", type=str, default="calibration_images")
    parser.add_argument("--output-dir", type=str, default="calibration_data")
    parser.add_argument("--url", type=str, default="rtsp://100.103.8.66:8554/video", help="RTSP URL")
    parser.add_argument("--debug", action="store_true", help="Log camera matrix")
    args = parser.parse_args()

    calib = load_calibration(args.calibration)
    if calib is None:
        return

    K = calib["camera_matrix"]
    dist = calib["dist_coeffs"]

    if args.debug:
        print("\nCamera Matrix:\n", K)
        print("\nDistortion Coefficients:", dist.flatten())

    if args.mode == "rtsp":
        pipeline = create_pipeline(args.url)
        print(f"Connecting to RTSP: {args.url}")
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    elif args.mode == "live":
        cap = cv2.VideoCapture(args.camera)
    else:
        cap = None

    if cap is not None and not cap.isOpened():
        print("Error: Cannot open camera/stream")
        return

    if args.mode == "images":
        images = sorted(Path(args.images_dir).glob("*.jpg"))
        out_path = Path(args.output_dir)
        print(f"Processing {len(images)} images...")
        
        # Precompute maps using first image size
        if images:
            sample = cv2.imread(str(images[0]))
            if sample is not None:
                h, w = sample.shape[:2]
                map1, map2, newK, roi = create_undistort_maps(K, dist, (w, h), alpha=0)
                if args.debug:
                    print(f"\nImage size: {w}x{h}")
                    print(f"New Camera Matrix:\n{newK}")
                    print(f"ROI: {roi}")
                
                for i, img_file in enumerate(images):
                    img = cv2.imread(str(img_file))
                    if img is None:
                        continue
                    undist = apply_undistort(img, map1, map2)
                    combined = np.hstack([img, undist])
                    cv2.putText(combined, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(combined, "Undistorted", (img.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    out_file = out_path / f"comparison_{img_file.name}"
                    cv2.imwrite(str(out_file), combined)
                    print(f"[{i+1}/{len(images)}] {out_file.name}")
                print(f"Saved to: {out_path}")
        return

    # Live/RTSP mode - compute maps once before loop
    print("ESC/Q to exit")
    cv2.namedWindow("Undistort", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Undistort", 1920, 1080)

    # Get frame size first
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read frame")
        cap.release()
        return

    h, w = frame.shape[:2]
    map1, map2, newK, roi = create_undistort_maps(K, dist, (w, h), alpha=0)

    if args.debug:
        print(f"\nImage size: {w}x{h}")
        print(f"New Camera Matrix:\n{newK}")
        print(f"ROI: {roi}")

    while True:
        undist = apply_undistort(frame, map1, map2)
        combined = np.hstack([frame, undist])
        cv2.imshow("Undistort", combined)

        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            break

        ret, frame = cap.read()
        if not ret:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
