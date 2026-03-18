# Interactive Calibration Tool - Modular Refactoring Summary

## Overview

The `interactive_calibrate.py` file has been successfully refactored from a single 568-line file into a clean, modular architecture. The functionality remains identical while the code is now much easier to understand, test, and extend.

## New Module Structure

```
src/camera_cal/
├── __init__.py                 # Module exports
├── config.py                   # Configuration parameters (66 lines)
├── camera.py                   # Camera stream management (100 lines)
├── calibrator.py               # Calibration core logic (250 lines)
├── ui.py                       # UI drawing functions (170 lines)
├── interactive_calibrate.py    # Main entry point (330 lines, refactored)
└── [other files unchanged]
```

## Module Descriptions

### 1. `config.py` - Configuration Management

**Purpose**: Centralized configuration for all calibration parameters

**Key Classes**:
- `CalibrationConfig`: Dataclass containing all configuration options
  - Chessboard pattern settings (pattern_size, square_size, min_samples)
  - Calibration parameters (calib_flags, alpha)
  - UI settings (window size, colors, capture cooldown)
  - Quality thresholds

**Usage Example**:
```python
from src.camera_cal.config import CalibrationConfig

# Use default config
config = CalibrationConfig()

# Customize specific settings
custom_config = CalibrationConfig(
    pattern_size=(10, 7),
    square_size=25.0,
    min_samples=15
)
```

### 2. `camera.py` - Camera Stream Management

**Purpose**: Unified interface for USB and RTSP camera sources

**Key Classes**:
- `CameraStream`: Handles video capture from USB or RTSP sources
  - `open()`: Initialize camera connection
  - `read()`: Read frames from camera
  - `release()`: Release camera resources
  - Context manager support (`with CameraStream(...) as cam:`)

**Usage Example**:
```python
from src.camera_cal.camera import CameraStream
from src.camera_cal.config import CalibrationConfig

config = CalibrationConfig()

# USB camera
usb_cam = CameraStream(camera_id=0, config=config)
usb_cam.open()

# RTSP stream
rtsp_cam = CameraStream(rtsp_url="rtsp://...", config=config)
rtsp_cam.open()
```

### 3. `calibrator.py` - Calibration Core Logic

**Purpose**: Complete calibration workflow implementation

**Key Classes**:
- `CameraCalibrator`: Main calibration engine
  - `detect_corners(gray)`: Detect and refine chessboard corners
  - `add_sample(corners, frame)`: Add calibration sample
  - `remove_last_sample()`: Remove most recent sample
  - `calibrate(image_size)`: Perform camera calibration
  - `create_undistort_maps(result)`: Create undistortion maps
  - `save_yaml(result, path)`: Save results to YAML
  - Static method `undistort(frame, map1, map2)`: Apply undistortion

- `CalibrationResult`: Dataclass for calibration results
  - camera_matrix, dist_coeffs, new_camera_matrix
  - roi, reprojection_error, image_size, num_samples

**Usage Example**:
```python
from src.camera_cal.calibrator import CameraCalibrator, CalibrationResult
from src.camera_cal.config import CalibrationConfig
import cv2

config = CalibrationConfig()
calibrator = CameraCalibrator(config)

# Detect chessboard
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
detected, corners = calibrator.detect_corners(gray)

# Add sample
if detected:
    calibrator.add_sample(corners, frame)

# Calibrate when enough samples
if calibrator.can_calibrate():
    result = calibrator.calibrate((gray.shape[1], gray.shape[0]))
```

### 4. `ui.py` - User Interface Drawing

**Purpose**: All visual feedback and overlay drawing

**Key Functions**:
- `draw_info_overlay()`: Main information overlay (mode, status, controls)
- `draw_chessboard_corners()`: Draw detected corner points
- `draw_side_by_side()`: Create original vs undistorted comparison
- `draw_pattern_hint()`: Draw pattern hint at bottom of frame

**Key Classes**:
- `CalibrationState`: Enum for CAPTURE and PREVIEW modes

**Usage Example**:
```python
from src.camera_cal.ui import (
    draw_info_overlay,
    draw_chessboard_corners,
    CalibrationState
)

# Draw overlay
draw_info_overlay(
    frame,
    CalibrationState.CAPTURE,
    sample_count=5,
    detected=True,
    config=config
)

# Draw corners
draw_chessboard_corners(frame, corners, config.pattern_size)
```

### 5. `interactive_calibrate.py` - Main Entry Point

**Purpose**: Coordinate all modules and handle main loop

**Key Changes**:
- Reduced from 568 lines to ~330 lines
- Extracted helper functions for better organization
- State machine simplified
- Delegates work to specialized modules

**Structure**:
- `main()`: Entry point, parses args, initializes components
- `_print_instructions()`: Display usage information
- `_handle_capture_state()`: Display logic for capture mode
- `_handle_preview_state()`: Display logic for preview mode
- `_handle_capture_keys()`: Keyboard input for capture mode
- `_handle_preview_keys()`: Keyboard input for preview mode

## Key Improvements

### 1. Separation of Concerns
- **Configuration**: Isolated in `config.py`
- **Camera I/O**: Isolated in `camera.py`
- **Calibration Logic**: Isolated in `calibrator.py`
- **UI Rendering**: Isolated in `ui.py`
- **Coordination**: Remains in `interactive_calibrate.py`

### 2. Testability
Each module can now be tested independently:
```python
# Test calibrator without camera
calibrator = CameraCalibrator(config)
calibrator.add_sample(mock_corners, mock_frame)
result = calibrator.calibrate((1280, 720))

# Test camera without calibrator
camera = CameraStream(camera_id=0)
camera.open()
ret, frame = camera.read()
```

### 3. Reusability
Modules can be imported and used independently:
```python
# Use calibrator in automated system
from src.camera_cal.calibrator import CameraCalibrator

# Use camera stream in other apps
from src.camera_cal.camera import CameraStream

# Customize configuration
from src.camera_cal.config import CalibrationConfig
```

### 4. Maintainability
- Each file has a single, clear responsibility
- Functions and classes are well-documented
- Easy to locate and fix bugs
- Simple to add new features

## Usage (Unchanged)

For end users, the usage remains exactly the same:

```bash
# Default RTSP
uv run icalib

# USB camera
uv run icalib --usb

# Custom camera ID
uv run icalib --usb --camera 1

# Custom RTSP URL
uv run icalib --url rtsp://192.168.1.100:8554/video

# Custom output file
uv run icalib --output my_calibration.yaml
```

## Extension Examples

The modular design makes it easy to add new features:

### Add ChArUco Board Support
```python
# In calibrator.py
class CameraCalibrator:
    def detect_charuco_corners(self, frame):
        # Add ChArUco detection logic
        pass
```

### Add Automated Capture
```python
# In interactive_calibrate.py
def auto_capture_quality_check(corners):
    # Implement quality scoring
    # Auto-capture if quality is good
    pass
```

### Add Different Calibration Patterns
```python
# In config.py
@dataclass
class CalibrationConfig:
    pattern_type: str = "chessboard"  # or "charuco", "circles"
```

## Compatibility

- **API**: Fully compatible with existing CLI interface
- **Output**: YAML format unchanged
- **Dependencies**: Same as before (opencv-python, numpy, pyyaml)
- **Python**: Requires Python 3.10+ for type hint improvements

## Testing Recommendations

### Unit Tests
```python
# Test configuration
def test_config_defaults():
    config = CalibrationConfig()
    assert config.pattern_size == (9, 6)
    assert config.min_samples == 12

# Test calibrator
def test_add_remove_samples():
    calibrator = CameraCalibrator()
    calibrator.add_sample(mock_corners, mock_frame)
    assert calibrator.get_sample_count() == 1
    calibrator.remove_last_sample()
    assert calibrator.get_sample_count() == 0
```

### Integration Tests
```python
# Test full workflow
def test_calibration_workflow():
    camera = CameraStream(camera_id=0)
    calibrator = CameraCalibrator()

    with camera:
        ret, frame = camera.read()
        detected, corners = calibrator.detect_corners(frame)
        # ... continue workflow
```

## Performance

The refactored code maintains the same performance characteristics:
- Real-time chessboard detection (~30 FPS)
- Fast undistortion using precomputed maps
- Minimal overhead from modularization

## Migration Notes

If you have code that imports from the old `interactive_calibrate.py`:

### Old Import (No Longer Works)
```python
from src.camera_cal.interactive_calibrate import (
    detect_chessboard,
    calibrate_camera,
    save_calibration_yaml,
)
```

### New Import (Correct Way)
```python
from src.camera_cal.calibrator import CameraCalibrator
from src.camera_cal.config import CalibrationConfig

config = CalibrationConfig()
calibrator = CameraCalibrator(config)
```

## Summary

This refactoring achieves all the stated goals:

✅ **Code Clarity**: Each module has a single, well-defined purpose
✅ **Testability**: Modules can be tested independently
✅ **Extensibility**: New features can be added without modifying existing code
✅ **Maintainability**: Bugs are easier to locate and fix
✅ **Backward Compatibility**: User interface remains unchanged

The codebase is now ready for:
- Adding automated capture features
- Supporting additional calibration patterns (ChArUco, asymmetric circles)
- Creating offline calibration tools
- Building web interfaces
- Writing comprehensive tests
