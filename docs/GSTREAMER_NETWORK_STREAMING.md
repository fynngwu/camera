# GStreamer Network Video Streaming Guide

This guide covers setting up network video streaming using GStreamer on Linux, with detailed explanations of each pipeline component.

## Environment Setup

### Installation (Ubuntu/Debian)

```bash
# Update package list
sudo apt update

# Install GStreamer core tools and plugins
sudo apt install gstreamer1.0-tools \
                 gstreamer1.0-plugins-base \
                 gstreamer1.0-plugins-good \
                 gstreamer1.0-plugins-bad \
                 gstreamer1.0-plugins-ugly \
                 gstreamer1.0-libav \
                 gstreamer1.0-alsa \
                 gstreamer1.0-pulseaudio

# Verify installation
gst-launch-1.0 --version
```

Expected output:
```
GStreamer 1.20.3
```

### Finding IP Addresses

```bash
# Show all IP addresses
ip addr show

# Or use
hostname -I
```

## H.264 Streaming (Recommended)

### Sender (Camera Machine)

```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! x264enc bitrate=2000 ! rtph264pay ! \
  udpsink host=192.168.1.100 port=5000
```

**Pipeline Breakdown:**

| Element | Purpose | Parameters |
|---------|---------|------------|
| `v4l2src` | Video capture from V4L2 device | `device=/dev/video0` specifies camera |
| `video/x-raw` | Caps filter for raw video format | Sets resolution to 1280x720@30fps |
| `videoconvert` | Color space conversion | Converts to format compatible with encoder |
| `x264enc` | H.264 video encoder | `bitrate=2000` sets 2000 kbps |
| `rtph264pay` | RTP packetizer | Encapsulates H.264 in RTP packets |
| `udpsink` | UDP network sender | `host` is receiver IP, `port` is receiver port |

**Adjusting Parameters:**

```bash
# Lower bitrate for slower networks (1000 kbps)
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! x264enc bitrate=1000 ! rtph264pay ! \
  udpsink host=192.168.1.100 port=5000

# Higher resolution (1920x1080)
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1920,height=1080,framerate=30/1 ! \
  videoconvert ! x264enc bitrate=4000 ! rtph264pay ! \
  udpsink host=192.168.1.100 port=5000

# Lower frame rate (15 fps)
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=15/1 ! \
  videoconvert ! x264enc bitrate=1500 ! rtph264pay ! \
  udpsink host=192.168.1.100 port=5000
```

### Receiver (Display Machine)

```bash
gst-launch-1.0 udpsrc port=5000 ! \
  application/x-rtp,media=video,encoding-name=H264 ! \
  rtph264depay ! h264parse ! avdec_h264 ! \
  videoconvert ! autovideosink
```

**Pipeline Breakdown:**

| Element | Purpose | Parameters |
|---------|---------|------------|
| `udpsrc` | UDP network receiver | `port=5000` listens for data |
| `application/x-rtp` | Caps filter for RTP format | Specifies H.264 video payload |
| `rtph264depay` | RTP depacketizer | Extracts H.264 from RTP packets |
| `h264parse` | H.264 stream parser | Ensures valid H.264 stream format |
| `avdec_h264` | H.264 decoder | Decodes using libav |
| `videoconvert` | Color space conversion | Converts to display format |
| `autovideosink` | Auto video output | Automatically selects best display |

## MJPEG Streaming (Low Latency Alternative)

MJPEG requires less CPU power and has lower latency but uses more network bandwidth.

### Sender (MJPEG)

```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  jpegenc quality=90 ! rtpjpegpay ! \
  udpsink host=192.168.1.100 port=5000
```

**Pipeline Breakdown:**

| Element | Purpose | Notes |
|---------|---------|-------|
| `jpegenc` | JPEG encoder | `quality=90` (1-100, higher=better) |
| `rtpjpegpay` | RTP JPEG packetizer | Encapsulates JPEG in RTP |

### Receiver (MJPEG)

```bash
gst-launch-1.0 udpsrc port=5000 ! \
  application/x-rtp,encoding-name=JPEG ! \
  rtpjpegdepay ! jpegdec ! \
  videoconvert ! autovideosink
```

## Saving Stream to File

### Save incoming stream to MP4

```bash
gst-launch-1.0 udpsrc port=5000 ! \
  application/x-rtp,media=video,encoding-name=H264 ! \
  rtph264depay ! h264parse ! \
  mp4mux ! filesink location=recording.mp4
```

### Save incoming stream to MKV

```bash
gst-launch-1.0 udpsrc port=5000 ! \
  application/x-rtp,media=video,encoding-name=H264 ! \
  rtph264depay ! h264parse ! \
  matroskamux ! filesink location=recording.mkv
```

## Network Configuration

### Wired Ethernet (Recommended)

For best results:
- Use Gigabit Ethernet (1000 Mbps)
- Connect both machines to the same switch/router
- Use dedicated network if possible

### WiFi

WiFi can work but expect:
- Lower effective bandwidth
- Occasional packet loss
- Higher latency

Reduce bitrate for WiFi:
```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! x264enc bitrate=800 ! rtph264pay ! \
  udpsink host=192.168.1.100 port=5000
```

### Port Forwarding (Across Networks)

To stream across different networks/routers:

1. **On the receiver's router**, forward port 5000 to the receiver machine
2. **Use the receiver's public IP** as the host
3. **Ensure firewall allows UDP traffic**

```bash
# On sender (use receiver's public IP)
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! x264enc bitrate=2000 ! rtph264pay ! \
  udpsink host=203.0.113.50 port=5000
```

## Troubleshooting

### No video on receiver

```bash
# Check network connectivity
ping 192.168.1.100

# Check if port is accessible
nc -uzv 192.168.1.100 5000

# Check UDP traffic
sudo tcpdump -i any port 5000
```

### Choppy video

Reduce resolution or bitrate:
```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=640,height=480,framerate=30/1 ! \
  videoconvert ! x264enc bitrate=1000 ! rtph264pay ! \
  udpsink host=192.168.1.100 port=5000
```

### High latency

Try MJPEG instead of H.264, or reduce encoding complexity:
```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! x264enc bitrate=1500 speed-preset=ultrafast ! rtph264pay ! \
  udpsink host=192.168.1.100 port=5000
```

### Display issues on receiver

Try specific video sink:
```bash
# For OpenGL accelerated display
gst-launch-1.0 udpsrc port=5000 ! \
  application/x-rtp,media=video,encoding-name=H264 ! \
  rtph264depay ! h264parse ! avdec_h264 ! \
  videoconvert ! glimagesink

# For X11 display
gst-launch-1.0 udpsrc port=5000 ! \
  application/x-rtp,media=video,encoding-name=H264 ! \
  rtph264depay ! h264parse ! avdec_h264 ! \
  videoconvert ! ximagesink
```

## Advanced: Bidirectional Streaming

Stream video both ways simultaneously:

```bash
# Machine A (sends to B, receives from B)
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! x264enc bitrate=2000 ! rtph264pay ! \
  udpsink host=192.168.1.100 port=5000 \
  udpsrc port=5001 ! \
  application/x-rtp,media=video,encoding-name=H264 ! \
  rtph264depay ! h264parse ! avdec_h264 ! \
  videoconvert ! autovideosink

# Machine B (sends to A, receives from A) - reverse ports
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! x264enc bitrate=2000 ! rtph264pay ! \
  udpsink host=192.168.1.50 port=5001 \
  udpsrc port=5000 ! \
  application/x-rtp,media=video,encoding-name=H264 ! \
  rtph264depay ! h264parse ! avdec_h264 ! \
  videoconvert ! autovideosink
```

## Quick Reference

| Resolution | Bitrate (Mbps) | Bandwidth Required |
|------------|----------------|-------------------|
| 640x480 | 1-2 | ~2 Mbps |
| 1280x720 | 2-4 | ~4 Mbps |
| 1920x1080 | 4-8 | ~8 Mbps |
| 3840x2160 | 15-30 | ~30 Mbps |
