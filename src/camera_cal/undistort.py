"""Undistort preview tool - view undistorted camera feed using calibration.yaml."""

import cv2
import numpy as np
import yaml
from pathlib import Path
from argparse import ArgumentParser


def load_calibration_yaml(path: str) -> dict:
    """Load camera calibration from YAML file.

    Args:
        path: YAML file path

    Returns:
        Dictionary with camera_matrix, dist_coeffs, new_camera_matrix, image_size, roi
    """
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    return {
        "camera_matrix": np.array(data["camera_matrix"]),
        "dist_coeffs": np.array(data["dist_coeffs"]),
        "new_camera_matrix": np.array(data["new_camera_matrix"]),
        "image_size": (data["image_width"], data["image_height"]),
        "roi": tuple(data["roi"]),
        "num_samples": data["num_samples"],
        "reprojection_error": data["mean_reprojection_error"],
    }


def create_undistort_maps(calib: dict, alpha: float = 1.0) -> tuple:
    """Create undistort maps.

    Args:
        calib: Calibration dictionary
        alpha: Alpha parameter (0 = all valid pixels, 1 = retain all original pixels)

    Returns:
        Tuple of (map1, map2, new_camera_matrix, roi)
    """
    w, h = calib["image_size"]
    newK, roi = cv2.getOptimalNewCameraMatrix(
        calib["camera_matrix"],
        calib["dist_coeffs"],
        (w, h),
        alpha,
        (w, h),
    )
    map1, map2 = cv2.initUndistortRectifyMap(
        calib["camera_matrix"],
        calib["dist_coeffs"],
        None,
        newK,
        (w, h),
        cv2.CV_16SC2,
    )
    return map1, map2, newK, roi


def apply_undistort(frame: np.ndarray, map1: np.ndarray, map2: np.ndarray) -> np.ndarray:
    """Apply undistortion using precomputed maps."""
    return cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)


def create_gstreamer_pipeline(url: str) -> str:
    """Create GStreamer pipeline for RTSP stream.

    Args:
        url: RTSP stream URL

    Returns:
        GStreamer pipeline string
    """
    return (
        f"rtspsrc location={url} latency=100 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! "
        "videoconvert ! appsink drop=true max-buffers=1 sync=false"
    )


def main():
    """Main entry point for undistort preview."""
    parser = ArgumentParser(description="View undistorted camera feed")
    parser.add_argument(
        "--calibration",
        type=str,
        default="calibration.yaml",
        help="Calibration YAML file path",
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera device ID (default: 0)",
    )
    parser.add_argument(
        "--url",
        type=str,
        default="rtsp://127.0.0.1:8554/video",
        help="RTSP stream URL",
    )
    parser.add_argument(
        "--usb",
        action="store_true",
        help="Use USB camera instead of RTSP",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=1.0,
        help="Alpha for undistort (0=all valid, 1=retain all)",
    )
    parser.add_argument(
        "--side-by-side",
        action="store_true",
        help="Show original and undistorted side by side",
    )
    args = parser.parse_args()

    # Load calibration
    calib_path = Path(args.calibration)
    if not calib_path.exists():
        print(f"Error: Calibration file not found: {calib_path}")
        return

    calib = load_calibration_yaml(str(calib_path))
    print(f"Loaded calibration: {calib_path}")
    print(f"  Image size: {calib['image_size']}")
    print(f"  Samples: {calib['num_samples']}")
    print(f"  Reprojection error: {calib['reprojection_error']:.4f} px")

    # Create undistort maps
    map1, map2, newK, roi = create_undistort_maps(calib, args.alpha)

    # Open video source using GStreamer pipeline
    if args.usb:
        cap = cv2.VideoCapture(args.camera)
        print(f"Using USB camera: {args.camera}")
    else:
        pipeline = create_gstreamer_pipeline(args.url)
        print(f"Using RTSP: {args.url}")
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Error: Cannot open video source")
        return

    # Create window
    window_name = "Undistort Preview"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)

    print("\nControls:")
    print("  U - Toggle undistort view")
    print("  S - Toggle side-by-side mode")
    print("  Q/ESC - Quit")
    print()

    show_undistorted = True
    side_by_side = args.side_by_side
    color_detected = (0, 255, 0)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Cannot read frame")
            break

        if show_undistorted:
            undistorted = apply_undistort(frame, map1, map2)

            if side_by_side:
                display = np.hstack([frame, undistorted])
                cv2.putText(display, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color_detected, 2)
                cv2.putText(display, "Undistorted", (frame.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color_detected, 2)
            else:
                display = undistorted
                cv2.putText(display, "Undistorted", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color_detected, 2)
        else:
            display = frame
            cv2.putText(display, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color_detected, 2)

        cv2.imshow(window_name, display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:  # Q or ESC
            break
        elif key == ord("u") or key == ord("U"):
            show_undistorted = not show_undistorted
            mode = "undistorted" if show_undistorted else "original"
            print(f"View mode: {mode}")
        elif key == ord("s") or key == ord("S"):
            side_by_side = not side_by_side
            print(f"Side-by-side: {side_by_side}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()