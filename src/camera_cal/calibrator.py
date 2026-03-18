"""Camera calibration core logic module.

This module encapsulates all camera calibration functionality including
chessboard detection, sample management, calibration computation,
and result storage.
"""

import cv2
import numpy as np
import yaml
from pathlib import Path
from dataclasses import dataclass
from typing import Tuple, Optional

from .config import CalibrationConfig


@dataclass
class CalibrationResult:
    """Camera calibration result."""

    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    new_camera_matrix: np.ndarray
    roi: tuple
    reprojection_error: float
    image_size: tuple
    num_samples: int


class CameraCalibrator:
    """Camera calibration engine.

    This class handles the complete calibration workflow:
    - Chessboard corner detection and refinement
    - Sample collection and management
    - Camera calibration computation
    - Undistortion map generation
    - Result serialization to YAML
    """

    def __init__(self, config: CalibrationConfig | None = None):
        """Initialize calibrator.

        Args:
            config: Calibration configuration (uses default if None)
        """
        self.config = config or CalibrationConfig()

        # Prepare object points for the chessboard pattern
        self._objp = self._create_object_points()

        # Storage for calibration data
        self._obj_points: list[np.ndarray] = []
        self._img_points: list[np.ndarray] = []
        self._captured_frames: list[np.ndarray] = []

    def _create_object_points(self) -> np.ndarray:
        """Create 3D object points for chessboard pattern.

        Returns:
            Array of 3D points representing the chessboard corners
        """
        objp = np.zeros(
            (self.config.pattern_size[0] * self.config.pattern_size[1], 3),
            dtype=np.float32,
        )
        objp[:, :2] = (
            np.mgrid[
                0 : self.config.pattern_size[0], 0 : self.config.pattern_size[1]
            ]
            .T.reshape(-1, 2)
            * self.config.square_size
        )
        return objp

    def detect_corners(
        self,
        gray: np.ndarray,
        scale: float = 1.0,
        precise: bool = False,
    ) -> Tuple[bool, Optional[np.ndarray]]:
        """Detect chessboard corners with optional downsampling.

        Args:
            gray: Grayscale image
            scale: Downscale factor (0.5 for preview, 1.0 for precise)
            precise: Whether to use precise detection (for capture)

        Returns:
            Tuple of (detected, corners)
        """
        # Downsample for preview
        if scale != 1.0:
            work = cv2.resize(
                gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )
        else:
            work = gray

        # Use SB algorithm for precise detection (faster for large images)
        if precise and hasattr(cv2, "findChessboardCornersSB"):
            ret, corners = cv2.findChessboardCornersSB(
                work,
                self.config.pattern_size,
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if ret and scale != 1.0:
                corners = corners / scale
            return ret, corners

        # Standard detection with FAST_CHECK for preview
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        if not precise:
            flags |= cv2.CALIB_CB_FAST_CHECK

        ret, corners = cv2.findChessboardCorners(work, self.config.pattern_size, flags)

        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(work, corners, (11, 11), (-1, -1), criteria)
            if scale != 1.0:
                corners = corners / scale
            return True, corners

        return False, None

    def add_sample(self, corners: np.ndarray, frame: np.ndarray) -> None:
        """Add a calibration sample.

        Args:
            corners: Detected chessboard corners (2D points)
            frame: Captured frame (for reference)
        """
        self._obj_points.append(self._objp.copy())
        self._img_points.append(corners)
        self._captured_frames.append(frame.copy())

    def remove_last_sample(self) -> bool:
        """Remove the last captured sample.

        Returns:
            True if a sample was removed, False if no samples exist
        """
        if self._obj_points:
            self._obj_points.pop()
            self._img_points.pop()
            self._captured_frames.pop()
            return True
        return False

    def get_sample_count(self) -> int:
        """Get the number of captured samples.

        Returns:
            Number of samples
        """
        return len(self._img_points)

    def can_calibrate(self) -> bool:
        """Check if calibration can be performed.

        Returns:
            True if enough samples are collected
        """
        return self.get_sample_count() >= self.config.min_samples

    def calibrate(self, image_size: Tuple[int, int]) -> Optional[CalibrationResult]:
        """Perform camera calibration with collected samples.

        Args:
            image_size: Image size (width, height)

        Returns:
            CalibrationResult if successful, None otherwise
        """
        if not self.can_calibrate():
            print(
                f"Need at least {self.config.min_samples} samples "
                f"(have {self.get_sample_count()})"
            )
            return None

        print("Calibrating camera...")

        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self._obj_points,
            self._img_points,
            image_size,
            None,
            None,
            flags=self.config.calib_flags,
        )

        # rms is the RMS reprojection error, not a boolean
        # Check if calibration succeeded by checking if outputs are valid
        if camera_matrix is None or not np.any(camera_matrix):
            print("Calibration failed!")
            return None

        # Calculate reprojection error
        mean_error = self._compute_reprojection_error(rvecs, tvecs, camera_matrix, dist_coeffs)

        # Compute optimal new camera matrix
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, image_size, self.config.alpha, image_size
        )

        print(f"OpenCV RMS error: {rms:.4f} pixels")
        print(f"Mean reprojection error: {mean_error:.4f} pixels")

        # Quality assessment
        if mean_error < self.config.quality_excellent:
            print("Quality: Excellent!")
        elif mean_error < self.config.quality_good:
            print("Quality: Good")
        elif mean_error < self.config.quality_acceptable:
            print("Quality: Acceptable")
        else:
            print("Quality: Needs improvement")

        return CalibrationResult(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            new_camera_matrix=new_camera_matrix,
            roi=roi,
            reprojection_error=mean_error,
            image_size=image_size,
            num_samples=self.get_sample_count(),
        )

    def _compute_reprojection_error(
        self,
        rvecs: list,
        tvecs: list,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> float:
        """Compute mean reprojection error.

        Args:
            rvecs: Rotation vectors
            tvecs: Translation vectors
            camera_matrix: Camera intrinsic matrix
            dist_coeffs: Distortion coefficients

        Returns:
            Mean reprojection error in pixels
        """
        total_error = 0.0
        total_points = 0

        for i in range(len(self._obj_points)):
            projected, _ = cv2.projectPoints(
                self._obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
            )
            error = cv2.norm(self._img_points[i], projected, cv2.NORM_L2) / len(
                projected
            )
            total_error += error * len(projected)
            total_points += len(projected)

        return total_error / total_points if total_points > 0 else 0.0

    def create_undistort_maps(
        self, result: CalibrationResult
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Create undistort maps for efficient real-time undistortion.

        Args:
            result: Calibration result

        Returns:
            Tuple of (map1, map2) for remapping
        """
        map1, map2 = cv2.initUndistortRectifyMap(
            result.camera_matrix,
            result.dist_coeffs,
            None,
            result.new_camera_matrix,
            result.image_size,
            cv2.CV_16SC2,
        )
        return map1, map2

    @staticmethod
    def undistort(
        frame: np.ndarray, map1: np.ndarray, map2: np.ndarray
    ) -> np.ndarray:
        """Apply undistortion using precomputed maps.

        Args:
            frame: Input frame
            map1: Undistort map 1
            map2: Undistort map 2

        Returns:
            Undistorted frame
        """
        return cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

    def save_yaml(self, result: CalibrationResult, output_path: Path) -> None:
        """Save calibration result to YAML file.

        Args:
            result: Calibration result
            output_path: Output file path
        """
        data = {
            "pattern_size": list(self.config.pattern_size),
            "square_size": self.config.square_size,
            "image_width": result.image_size[0],
            "image_height": result.image_size[1],
            "camera_matrix": result.camera_matrix.tolist(),
            "dist_coeffs": result.dist_coeffs.ravel().tolist(),
            "new_camera_matrix": result.new_camera_matrix.tolist(),
            "roi": list(result.roi),
            "num_samples": result.num_samples,
            "mean_reprojection_error": float(result.reprojection_error),
        }

        with open(output_path, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        print(f"Calibration saved to: {output_path}")
