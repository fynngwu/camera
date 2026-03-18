"""Camera calibration module.

This module performs camera calibration using chessboard pattern images.
It computes intrinsic camera parameters and distortion coefficients.
"""

import cv2
import numpy as np
from pathlib import Path
import argparse


# Chessboard pattern configuration
CHESSBOARD_COLS = 9  # Inner corners in width
CHESSBOARD_ROWS = 6  # Inner corners in height


def calibrate_camera(
    images_dir: str = "calibration_images",
    output_dir: str = "calibration_data",
    square_size: float = 30.0,
) -> tuple[bool, float]:
    """Perform camera calibration using chessboard images.

    Args:
        images_dir: Directory containing calibration images
        output_dir: Directory to save calibration results
        square_size: Actual size of chessboard square in millimeters

    Returns:
        Tuple of (success, reprojection_error)
    """
    # Create output directory
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True)

    # Load images
    images_path = Path(images_dir)
    image_files = sorted(images_path.glob("*.jpg")) + sorted(images_path.glob("*.png"))

    if len(image_files) < 10:
        print(f"Warning: Only {len(image_files)} images found.")
        print("Recommendation: Capture at least 15-20 images for good calibration.")

    print("=" * 50)
    print("Camera Calibration")
    print("=" * 50)
    print(f"Pattern: {CHESSBOARD_COLS}x{CHESSBOARD_ROWS} inner corners")
    print(f"Square size: {square_size} mm")
    print(f"Images: {len(image_files)}")
    print("=" * 50)
    print()

    # Prepare object points (3D world coordinates)
    # The chessboard is on the Z=0 plane
    objp = np.zeros((CHESSBOARD_COLS * CHESSBOARD_ROWS, 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[
        0:CHESSBOARD_COLS, 0:CHESSBOARD_ROWS
    ].T.reshape(-1, 2) * square_size

    # Arrays to store object points and image points
    obj_points = []  # 3D points in real world space
    img_points = []  # 2D points in image plane

    # Process each image
    valid_images = []
    failed_images = []

    for i, image_file in enumerate(image_files):
        img = cv2.imread(str(image_file))
        if img is None:
            print(f"[{i+1:3d}/{len(image_files)}] Failed to load: {image_file.name}")
            failed_images.append(image_file.name)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(
            gray,
            (CHESSBOARD_COLS, CHESSBOARD_ROWS),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK,
        )

        if ret:
            # Refine corner positions
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001,
            )
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )

            obj_points.append(objp)
            img_points.append(corners_refined)
            valid_images.append(image_file.name)

            # Draw and save visualization
            vis = img.copy()
            cv2.drawChessboardCorners(
                vis, (CHESSBOARD_COLS, CHESSBOARD_ROWS), corners_refined, ret
            )
            vis_path = output_path / f"detected_{image_file.stem}.jpg"
            cv2.imwrite(str(vis_path), vis)

            print(
                f"[{i+1:3d}/{len(image_files)}] ✓ {image_file.name} - {len(corners_refined)} corners"
            )
        else:
            failed_images.append(image_file.name)
            print(f"[{i+1:3d}/{len(image_files)}] ✗ {image_file.name} - No pattern found")

    print()
    print(f"Valid images: {len(valid_images)}/{len(image_files)}")

    if len(valid_images) < 5:
        print("Error: Need at least 5 valid images for calibration")
        return False, 0.0

    # Perform calibration
    print()
    print("Calibrating camera...")
    img_shape = gray.shape[::-1]

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points,
        img_points,
        img_shape,
        None,
        None,
        flags=cv2.CALIB_FIX_K3,  # Don't use radial distortion coefficient k3
    )

    if not ret:
        print("Error: Camera calibration failed")
        return False, 0.0

    # Calculate reprojection error
    total_error = 0
    for i in range(len(obj_points)):
        img_points_reprojected, _ = cv2.projectPoints(
            obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
        )
        error = cv2.norm(img_points[i], img_points_reprojected, cv2.NORM_L2) / len(
            img_points_reprojected
        )
        total_error += error

    mean_error = total_error / len(obj_points)

    print()
    print("=" * 50)
    print("Calibration Results")
    print("=" * 50)
    print(f"Reprojection Error: {mean_error:.4f} pixels")
    print()
    print("Camera Matrix:")
    print(camera_matrix)
    print()
    print("Distortion Coefficients:")
    print(dist_coeffs.ravel())
    print("=" * 50)
    print()

    # Save calibration data
    output_file = output_path / "camera_calibration.npz"
    np.savez(
        output_file,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        rvecs=rvecs,
        tvecs=tvecs,
        reprojection_error=mean_error,
        image_size=img_shape,
        chessboard_cols=CHESSBOARD_COLS,
        chessboard_rows=CHESSBOARD_ROWS,
        square_size=square_size,
    )
    print(f"Calibration data saved to: {output_file.absolute()}")
    print()

    # Quality assessment
    if mean_error < 0.3:
        print("Quality: Excellent calibration!")
    elif mean_error < 0.5:
        print("Quality: Good calibration.")
    elif mean_error < 1.0:
        print("Quality: Acceptable calibration. Consider capturing more images.")
    else:
        print("Quality: Poor calibration. Recapture with better images.")

    return True, mean_error


def main():
    """Main entry point for camera calibration."""
    parser = argparse.ArgumentParser(
        description="Calibrate camera using chessboard images"
    )
    parser.add_argument(
        "--images",
        type=str,
        default="calibration_images",
        help="Directory containing calibration images",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="calibration_data",
        help="Output directory for calibration results",
    )
    parser.add_argument(
        "--square-size",
        type=float,
        default=30.0,
        help="Actual size of chessboard square in millimeters",
    )
    args = parser.parse_args()

    success, error = calibrate_camera(
        images_dir=args.images,
        output_dir=args.output,
        square_size=args.square_size,
    )

    if not success:
        exit(1)


if __name__ == "__main__":
    main()
