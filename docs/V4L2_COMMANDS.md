# V4L2-ctl Commands Reference

This document provides a quick reference for commonly used `v4l2-ctl` commands for inspecting and configuring USB cameras on Linux.

## Installation

```bash
sudo apt install v4l-utils
```

## Listing Devices

### View all video devices
```bash
v4l2-ctl --list-devices
```

Example output:
```
USB Camera (usb-0000:00:14.0-1):
    /dev/video0
    /dev/video1

Integrated Camera (usb-0000:00:14.0-2):
    /dev/video2
```

## Device Information

### View all device properties
```bash
v4l2-ctl -d /dev/video0 --all
```

This shows:
- Device name and driver
- Supported video formats
- Current resolution and frame rate
- All controls (brightness, contrast, exposure, etc.)

### List supported formats
```bash
v4l2-ctl -d /dev/video0 --list-formats
```

Example output:
```
ioctl: VIDIOC_ENUM_FMT
    Type: Video Capture

    [0]: 'YUYV' (YUYV 4:2:2)
    [1]: 'MJPG' (Motion-JPEG)
```

### List supported resolutions and frame rates
```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

Example output:
```
    [0]: 'YUYV' (YUYV 4:2:2)
        Size: Discrete 640x480
            Interval: Discrete 0.033s (30.000 fps)
            Interval: Discrete 0.050s (20.000 fps)
        Size: Discrete 1280x720
            Interval: Discrete 0.033s (30.000 fps)
```

## Camera Controls

### View all control values
```bash
v4l2-ctl -d /dev/video0 --list-ctrls
```

Example output:
```
    brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
    contrast 0x00980901 (int)      : min=0 max=64 step=1 default=32 value=32
    saturation 0x00980902 (int)    : min=0 max=128 step=1 default=64 value=64
    exposure_absolute 0x009a0902 (int): min=1 max=5000 step=1 default=156 value=156
```

### Get specific control value
```bash
v4l2-ctl -d /dev/video0 --get-ctrl=brightness
v4l2-ctl -d /dev/video0 --get-ctrl=exposure_absolute
```

### Set control values
```bash
# Set brightness (-64 to 64)
v4l2-ctl -d /dev/video0 --set-ctrl=brightness=10

# Set contrast (0 to 64)
v4l2-ctl -d /dev/video0 --set-ctrl=contrast=40

# Set exposure time in microseconds (1 to 5000)
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=100

# Enable/disable auto exposure (1=on, 0=off)
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1

# Set white balance temperature (2000 to 6500K)
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature_auto=0
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=4500
```

## Video Format Configuration

### Set resolution and format
```bash
# Set to 1280x720 YUYV format
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=720,pixelformat=YUYV

# Set to 1920x1080 MJPEG format
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=MJPG
```

### Get current format
```bash
v4l2-ctl -d /dev/video0 --get-fmt-video
```

### Set frame rate
```bash
# Set to 30 fps
v4l2-ctl -d /dev/video0 --set-parm=30
```

### Get current frame rate
```bash
v4l2-ctl -d /dev/video0 --get-parm
```

## Troubleshooting

### Check if camera is accessible
```bash
# List all video devices
ls -l /dev/video*

# Check device permissions
v4l2-ctl -d /dev/video0 --all

# Test with a simple capture
v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=1 --stream-to=test.raw
```

### Common issues

**Permission denied:**
```bash
# Add user to video group
sudo usermod -a -G video $USER
# Log out and back in for changes to take effect
```

**Device busy:**
```bash
# Check what's using the camera
sudo lsof /dev/video0

# Stop any running camera applications
```

## Pixel Format Codes

| FourCC | Description |
|--------|-------------|
| YUYV | YUYV 4:2:2 (uncompressed) |
| RGB3 | RGB 24-bit |
| MJPG | Motion-JPEG (compressed) |
| H264 | H.264 (compressed) |

## Tips for This Project

When using this camera calibration toolkit:

1. **Check supported formats** before capturing to ensure compatibility
2. **Set consistent resolution** - the calibration assumes constant resolution
3. **Disable auto-focus** if available for more consistent calibration
4. **Lock exposure** to prevent changes during calibration image capture
5. **Use MJPEG** if you need higher resolution at lower USB bandwidth

```bash
# Recommended settings for calibration
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=720,pixelformat=MJPG
v4l2-ctl -d /dev/video0 --set-parm=30
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1
```
