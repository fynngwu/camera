"""Interactive camera calibration tool.

This module provides an all-in-one interactive tool for camera calibration.
It integrates capture, calibration, and preview in a single application with
real-time feedback and intuitive keyboard controls.

This is the main entry point that coordinates all modules.
"""

import cv2
import numpy as np
import time
from pathlib import Path
from argparse import ArgumentParser

from .config import CalibrationConfig
from .camera import CameraStream
from .calibrator import CameraCalibrator, CalibrationResult
from .ui import (
    CalibrationState,
    draw_info_overlay,
    draw_chessboard_corners,
    draw_pattern_hint,
)


def main():
    """Main entry point for interactive calibration tool."""
    # Parse command line arguments
    parser = ArgumentParser(description="Interactive camera calibration tool")
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera device ID (default: 0)",
    )
    parser.add_argument(
        "--url",
        type=str,
        default="rtsp://100.103.8.66:8554/video",
        help="RTSP stream URL",
    )
    parser.add_argument(
        "--usb",
        action="store_true",
        help="Use USB camera instead of RTSP",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="calibration.yaml",
        help="Output YAML file path",
    )
    parser.add_argument(
        "--save-images",
        type=str,
        default=None,
        help="Directory to save captured images (default: images)",
    )
    args = parser.parse_args()

    # Initialize configuration
    config = CalibrationConfig()

    # Create image save directory
    image_save_dir = Path(args.save_images) if args.save_images else Path("images")
    image_save_dir.mkdir(parents=True, exist_ok=True)
    print(f"Images will be saved to: {image_save_dir}")

    # Initialize state
    state = CalibrationState.CAPTURE

    # Initialize calibrator
    calibrator = CameraCalibrator(config)

    # Calibration result and undistortion
    calibration_result: CalibrationResult | None = None
    undistort_maps: tuple[np.ndarray, np.ndarray] | None = None
    show_undistorted = True
    saved = False

    # Capture cooldown
    last_capture_time = 0

    # Open video source
    camera = CameraStream(
        camera_id=args.camera, rtsp_url=None if args.usb else args.url, config=config
    )

    if not camera.open():
        return

    # Create window
    cv2.namedWindow(config.window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(config.window_name, config.window_width, config.window_height)

    # Print instructions
    _print_instructions(config)

    # Main loop with frame counter for skip detection
    frame_idx = 0
    preview_detected = False
    preview_corners = None
    corners_frame_idx = -1  # Track which frame the corners belong to

    while True:
        ret, frame = camera.read()
        if not ret:
            print("Error: Cannot read frame")
            break

        display_frame = frame.copy()
        frame_idx += 1

        if state == CalibrationState.CAPTURE:
            # Skip frame detection for performance
            should_detect = (
                config.detect_every_n_frames == 0
                or frame_idx % config.detect_every_n_frames == 0
            )

            if should_detect:
                # Only convert to grayscale, let detect_corners handle scaling
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                preview_detected, preview_corners = calibrator.detect_corners(
                    gray,
                    scale=config.preview_scale,
                    precise=False,
                )
                corners_frame_idx = frame_idx  # Update frame index for these corners

            display_frame = _handle_capture_state(
                display_frame,
                preview_detected,
                preview_corners,
                corners_frame_idx,
                frame_idx,
                calibrator,
                config,
            )

        elif state == CalibrationState.PREVIEW:
            display_frame = _handle_preview_state(
                display_frame,
                frame,
                undistort_maps,
                show_undistorted,
                saved,
                config,
            )

        cv2.imshow(config.window_name, display_frame)

        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:  # Q or ESC
            break

        elif state == CalibrationState.CAPTURE:
            new_state, result, maps, last_capture_time = _handle_capture_keys(
                key,
                frame,
                calibrator,
                config,
                last_capture_time,
                image_save_dir,
            )
            if new_state is not None:
                state = new_state
            if result is not None:
                calibration_result = result
            if maps is not None:
                undistort_maps = maps

        elif state == CalibrationState.PREVIEW:
            show_undistorted, saved = _handle_preview_keys(
                key,
                calibration_result,
                calibrator,
                args.output,
                show_undistorted,
            )

    # Cleanup
    camera.release()
    cv2.destroyAllWindows()

    if calibration_result is not None and not saved:
        print("\nCalibration completed but not saved.")
        print(f"Run again and press 'S' to save to {args.output}")


def _print_instructions(config: CalibrationConfig) -> None:
    """Print calibration instructions.

    Args:
        config: Calibration configuration
    """
    print("=" * 50)
    print("Interactive Camera Calibration")
    print("=" * 50)
    print(f"Pattern: {config.pattern_size[0]}x{config.pattern_size[1]} inner corners")
    print(f"Square size: {config.square_size} mm")
    print(f"Minimum samples: {config.min_samples}")
    print()
    print("Controls:")
    print("  C - Capture current frame")
    print("  D - Delete last capture")
    print("  ENTER - Start calibration")
    print("  U - Toggle undistort (preview mode)")
    print("  S - Save calibration result")
    print("  Q - Quit")
    print("=" * 50)


def _handle_capture_state(
    display_frame: np.ndarray,
    detected: bool,
    corners: np.ndarray | None,
    corners_frame_idx: int,
    current_frame_idx: int,
    calibrator: CameraCalibrator,
    config: CalibrationConfig,
) -> np.ndarray:
    """Handle display in capture state.

    Args:
        display_frame: Frame to display
        detected: Whether chessboard was detected
        corners: Detected corners (if any)
        corners_frame_idx: Frame index when corners were detected
        current_frame_idx: Current frame index
        calibrator: Calibrator instance
        config: Configuration

    Returns:
        Updated display frame
    """
    # Only draw corners if they belong to the current frame (prevents stale corner drift)
    if detected and corners is not None and corners_frame_idx == current_frame_idx:
        draw_chessboard_corners(display_frame, corners, config.pattern_size)

    # Draw overlay
    draw_info_overlay(
        display_frame,
        CalibrationState.CAPTURE,
        calibrator.get_sample_count(),
        detected,
        config,
    )

    # Draw hint at bottom
    draw_pattern_hint(display_frame, config.pattern_size, config)

    return display_frame


def _handle_preview_state(
    display_frame: np.ndarray,
    frame: np.ndarray,
    undistort_maps: tuple[np.ndarray, np.ndarray] | None,
    show_undistorted: bool,
    saved: bool,
    config: CalibrationConfig,
) -> np.ndarray:
    """Handle display in preview state.

    Shows single image (original or undistorted) instead of side-by-side.

    Args:
        display_frame: Frame to display
        frame: Original frame
        undistort_maps: Undistortion maps
        show_undistorted: Whether to show undistorted view
        saved: Whether calibration has been saved
        config: Configuration

    Returns:
        Updated display frame
    """
    # Apply undistortion if available
    if undistort_maps is not None and show_undistorted:
        map1, map2 = undistort_maps
        display_frame = CameraCalibrator.undistort(frame, map1, map2)
        cv2.putText(
            display_frame,
            "Undistorted",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            config.color_detected,
            2,
        )
    else:
        display_frame = frame
        cv2.putText(
            display_frame,
            "Original",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            config.color_detected,
            2,
        )

    # Show saved status
    if saved:
        width = display_frame.shape[1]
        cv2.putText(
            display_frame,
            "Saved!",
            (width - 150, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

    return display_frame


def _handle_capture_keys(
    key: int,
    frame: np.ndarray,
    calibrator: CameraCalibrator,
    config: CalibrationConfig,
    last_capture_time: float,
    image_save_dir: Path,
) -> tuple:
    """Handle keyboard input in capture state.

    Args:
        key: Key code
        frame: Color frame
        calibrator: Calibrator instance
        config: Configuration
        last_capture_time: Last capture timestamp
        image_save_dir: Directory to save captured images

    Returns:
        Tuple of (new_state, calibration_result, undistort_maps, updated_last_capture_time)
    """
    new_state = None
    result = None
    maps = None
    updated_last_capture_time = last_capture_time  # Track updated time

    if key == ord("c") or key == ord("C"):  # Capture
        current_time = time.time()
        if current_time - last_capture_time >= config.capture_cooldown:
            # Full resolution precise detection for capture
            gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detected, corners = calibrator.detect_corners(
                gray_full,
                scale=1.0,
                precise=True,
            )

            if detected:
                sample_count = calibrator.get_sample_count() + 1
                calibrator.add_sample(corners, frame)
                updated_last_capture_time = current_time  # Update last capture time on success

                # Save image with chessboard corners drawn
                save_frame = frame.copy()
                cv2.drawChessboardCorners(save_frame, config.pattern_size, corners, True)
                image_path = image_save_dir / f"capture_{sample_count:03d}.png"
                cv2.imwrite(str(image_path), save_frame)
                print(f"[{sample_count:3d}] Captured frame -> {image_path}")

                # Flash effect
                frame_bright = cv2.convertScaleAbs(frame, alpha=1.5, beta=50)
                cv2.imshow(config.window_name, frame_bright)
                cv2.waitKey(50)
            else:
                print("Capture failed: No chessboard detected")

    elif key == ord("d") or key == ord("D"):  # Delete last
        sample_count = calibrator.get_sample_count()
        if calibrator.remove_last_sample():
            # Delete corresponding image file
            image_path = image_save_dir / f"capture_{sample_count:03d}.png"
            if image_path.exists():
                image_path.unlink()
            print(f"Deleted last capture. Remaining: {calibrator.get_sample_count()}")

    elif key == 13:  # ENTER - Start calibration
        if not calibrator.can_calibrate():
            print(
                f"Need at least {config.min_samples} samples "
                f"(have {calibrator.get_sample_count()})"
            )
            return new_state, result, maps, updated_last_capture_time

        img_size = (frame.shape[1], frame.shape[0])
        calibration_result = calibrator.calibrate(img_size)

        if calibration_result is not None:
            result = calibration_result
            maps = calibrator.create_undistort_maps(calibration_result)
            new_state = CalibrationState.PREVIEW
            print("Calibration complete! Press U to toggle undistort, S to save.")
        else:
            print("Calibration failed. Please try again.")

    return new_state, result, maps, updated_last_capture_time


def _handle_preview_keys(
    key: int,
    calibration_result: CalibrationResult | None,
    calibrator: CameraCalibrator,
    output_path: str,
    show_undistorted: bool,
) -> tuple[bool, bool]:
    """Handle keyboard input in preview state.

    Args:
        key: Key code
        calibration_result: Calibration result
        calibrator: Calibrator instance (preserves config)
        output_path: Output file path
        show_undistorted: Current undistort display state

    Returns:
        Tuple of (show_undistorted, saved)
    """
    saved = False

    if key == ord("u") or key == ord("U"):  # Toggle undistort
        show_undistorted = not show_undistorted
        mode = "undistorted" if show_undistorted else "original"
        print(f"Display mode: {mode}")

    elif key == ord("s") or key == ord("S"):  # Save
        if calibration_result is not None:
            calibrator.save_yaml(calibration_result, Path(output_path))
            saved = True

    return show_undistorted, saved


if __name__ == "__main__":
    main()
