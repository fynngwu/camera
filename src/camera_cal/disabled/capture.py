"""Camera calibration image capture module.

This module provides functionality to capture calibration images from a USB camera or RTSP stream.
It displays a live preview and allows the user to save frames by pressing SPACE.
"""

import cv2
import time
from pathlib import Path
import argparse


def create_pipeline(url: str) -> str:
    """Create GStreamer pipeline for RTSP stream.

    Args:
        url: RTSP stream URL

    Returns:
        GStreamer pipeline string
    """
    return (
        f"rtspsrc location={url} latency=0 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! "
        "videoconvert ! appsink"
    )


def capture_images(
    camera_id: int = 0,
    output_dir: str = "calibration_images",
    preview_width: int = 640,
    preview_height: int = 480,
    rtsp_url: str = None,
) -> None:
    """Capture calibration images from a USB camera or RTSP stream.

    Args:
        camera_id: Camera device ID (default: 0 for /dev/video0)
        output_dir: Directory to save captured images
        preview_width: Preview window width
        preview_height: Preview window height
        rtsp_url: RTSP stream URL (if None, use USB camera; if provided, use RTSP stream)
    """
    # Create output directory
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True)

    # Count existing images
    existing_images = list(output_path.glob("*.jpg"))
    image_count = len(existing_images)

    # Open camera or RTSP stream
    if rtsp_url:
        pipeline = create_pipeline(rtsp_url)
        print(f"Connecting to RTSP stream: {rtsp_url}")
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            gst_enabled = "GStreamer:                   YES" in cv2.getBuildInformation()
            print(f"Error: Cannot open RTSP stream via GStreamer: {rtsp_url}")
            print(f"OpenCV GStreamer support: {'YES' if gst_enabled else 'NO'}")
            print("Try testing the stream with:")
            print(f"  gst-launch-1.0 rtspsrc location={rtsp_url} latency=0 ! "
                  "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink")
            return
        print(f"Connected to RTSP stream via GStreamer: {rtsp_url}")
    else:
        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            print(f"Error: Cannot open camera {camera_id}")
            print("Please check:")
            print("  - Camera is connected")
            print("  - Camera device ID is correct")
            print("  - You have permission to access the camera")
            return
        print(f"Using camera: /dev/video{camera_id}")
        # Set camera properties for USB camera
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("=" * 50)
    print("Camera Calibration Image Capture")
    print("=" * 50)
    if rtsp_url:
        print(f"Stream: {rtsp_url}")
    else:
        print(f"Camera: /dev/video{camera_id}")
    print(f"Output: {output_path.absolute()}")
    print()
    print("Controls:")
    print("  SPACE - Capture current frame")
    print("  ESC/Q - Exit")
    print("=" * 50)
    print()

    window_name = "Camera Preview - Capture"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, preview_width, preview_height)

    last_capture_time = 0
    min_capture_interval = 0.5  # Minimum seconds between captures

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Cannot read from camera")
            break

        # Display info overlay
        info_text = [
            f"Images: {image_count}",
            f"Resolution: {frame.shape[1]}x{frame.shape[0]}",
            "SPACE: Capture | ESC/Q: Exit",
        ]

        for i, text in enumerate(info_text):
            y = 30 + i * 30
            cv2.putText(
                frame,
                text,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        # Draw corner count hint
        hint_text = "Chessboard: 8x5 inner corners"
        cv2.putText(
            frame,
            hint_text,
            (10, frame.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF

        # Capture on SPACE
        if key == ord(" "):
            current_time = time.time()
            if current_time - last_capture_time >= min_capture_interval:
                filename = output_path / f"calib_{image_count:04d}.jpg"
                cv2.imwrite(str(filename), frame)
                image_count += 1
                last_capture_time = current_time
                print(f"[{image_count:3d}] Saved: {filename.name}")

                # Flash effect
                frame_bright = cv2.convertScaleAbs(frame, alpha=1.5, beta=50)
                cv2.imshow(window_name, frame_bright)
                cv2.waitKey(50)

        # Exit on ESC or Q
        elif key == 27 or key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print()
    print(f"Capture complete. Total images: {image_count}")
    print(f"Images saved to: {output_path.absolute()}")


def main():
    """Main entry point for image capture."""
    parser = argparse.ArgumentParser(
        description="Capture calibration images from USB camera or RTSP stream"
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera device ID (default: 0 for /dev/video0)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="calibration_images",
        help="Output directory for captured images",
    )
    parser.add_argument(
        "-u", "--url",
        type=str,
        default="rtsp://100.103.8.66:8554/video",
        help="RTSP stream URL. If provided, use RTSP instead of USB camera. (default: rtsp://100.103.8.66:8554/video)",
    )
    parser.add_argument(
        "--usb",
        action="store_true",
        help="Force use USB camera instead of RTSP stream",
    )
    args = parser.parse_args()

    # Use RTSP by default, but use USB camera if --usb flag is provided
    rtsp_url = None if args.usb else args.url

    capture_images(
        camera_id=args.camera,
        output_dir=args.output,
        rtsp_url=rtsp_url,
    )


if __name__ == "__main__":
    main()
