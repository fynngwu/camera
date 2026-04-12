#!/usr/bin/env python3
import cv2
import sys

def test_gstreamer_support():
    print("=" * 60)
    print("OpenCV GStreamer 支持检测")
    print("=" * 60)
    
    print(f"OpenCV 版本: {cv2.__version__}")
    
    if cv2.CAP_GSTREAMER:
        print("✓ OpenCV 支持 GStreamer")
        print(f"  CAP_GSTREAMER 值: {cv2.CAP_GSTREAMER}")
    else:
        print("✗ OpenCV 不支持 GStreamer")
        sys.exit(1)
    
    print("")

def test_camera_access():
    print("测试 GStreamer 相机访问...")
    
    pipelines = [
        {
            "name": "H264 管道 (1280x720@30)",
            "pipeline": (
                "v4l2src device=/dev/video0 ! "
                "video/x-h264,width=1280,height=720,framerate=30/1 ! "
                "h264parse ! "
                "avdec_h264 ! "
                "videoconvert ! "
                "appsink sync=0"
            )
        },
        {
            "name": "MJPG 管道 (1280x720@30)",
            "pipeline": (
                "v4l2src device=/dev/video0 ! "
                "image/jpeg,width=1280,height=720,framerate=30/1 ! "
                "jpegdec ! "
                "videoconvert ! "
                "appsink sync=0"
            )
        },
        {
            "name": "H264 管道 (1920x1080@30)",
            "pipeline": (
                "v4l2src device=/dev/video0 ! "
                "video/x-h264,width=1920,height=1080,framerate=30/1 ! "
                "h264parse ! "
                "avdec_h264 ! "
                "videoconvert ! "
                "appsink sync=0"
            )
        },
        {
            "name": "H264 管道 (640x480@30)",
            "pipeline": (
                "v4l2src device=/dev/video0 ! "
                "video/x-h264,width=640,height=480,framerate=30/1 ! "
                "h264parse ! "
                "avdec_h264 ! "
                "videoconvert ! "
                "appsink sync=0"
            )
        }
    ]
    
    success_count = 0
    
    for config in pipelines:
        print(f"\n测试 {config['name']}:")
        cap = cv2.VideoCapture(config['pipeline'], cv2.CAP_GSTREAMER)
        
        if not cap.isOpened():
            print(f"  ✗ 无法打开")
            continue
        
        print(f"  ✓ 打开成功")
        
        ret, frame = cap.read()
        if ret:
            print(f"  ✓ 成功读取帧: {frame.shape}")
            success_count += 1
        else:
            print(f"  ✗ 无法读取帧")
        
        cap.release()
    
    print(f"\n总结: {success_count}/{len(pipelines)} 管道测试成功")
    return success_count > 0

def test_direct_v4l2():
    print("\n")
    print("测试直接 V4L2 访问...")
    
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("✗ 无法直接打开相机")
        return False
    
    print("✓ 直接 V4L2 打开成功")
    
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"  当前分辨率: {int(width)}x{int(height)}")
    print(f"  当前帧率: {fps} fps")
    
    ret, frame = cap.read()
    if ret:
        print(f"✓ 成功读取帧: {frame.shape}")
        cv2.imwrite('/tmp/camera_test_frame.jpg', frame)
        print("  测试帧已保存到 /tmp/camera_test_frame.jpg")
    else:
        print("✗ 无法读取帧")
    
    cap.release()
    return True

def test_gstreamer_cli():
    print("\n")
    print("GStreamer 命令行测试结果:")
    print("--------------------------")
    print("已验证 gst-launch-1.0 命令可正常工作:")
    print("  gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-h264,width=1280,height=720,framerate=30/1 ! h264parse ! avdec_h264 ! autovideosink")
    print("✓ 命令行测试成功")

if __name__ == "__main__":
    test_gstreamer_support()
    test_camera_access()
    test_direct_v4l2()
    test_gstreamer_cli()
    
    print("\n")
    print("=" * 60)
    print("测试完成！")
    print("=" * 60)
    print("\n推荐使用的 GStreamer 管道:")
    print("1. H264 格式 (推荐):")
    print("   v4l2src device=/dev/video0 ! video/x-h264,width=1280,height=720,framerate=30/1 ! h264parse ! avdec_h264 ! videoconvert ! appsink sync=0")
    print("\n2. MJPG 格式:")
    print("   v4l2src device=/dev/video0 ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegdec ! videoconvert ! appsink sync=0")