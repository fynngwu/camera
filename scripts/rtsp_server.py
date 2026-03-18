#!/usr/bin/env python3
"""
RTSP 服务器 - 支持命令行配置分辨率和帧率
用法：
    python3 rtsp_server.py                    # 默认 720p30
    python3 rtsp_server.py --width 1920 --height 1080 --fps 30   # 1080p30
    python3 rtsp_server.py --width 640 --height 480 --fps 60     # VGA 60fps
"""

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import GLib, Gst, GstRtspServer
import sys
import argparse

def parse_args():
    parser = argparse.ArgumentParser(
        description='GStreamer RTSP 摄像头服务器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                              # 默认 720p30
  %(prog)s -w 1920 -H 1080 -f 30        # 1080p30
  %(prog)s -w 1280 -H 720 -f 60         # 720p60
  %(prog)s -w 640 -H 480 -f 30          # VGA 30fps
  %(prog)s --device /dev/video1         # 使用其他摄像头
        """
    )
    
    parser.add_argument('-w', '--width', type=int, default=1920,
                       help='视频宽度 (默认：1920)')
    parser.add_argument('-H', '--height', type=int, default=1080,
                       help='视频高度 (默认：1080)')
    parser.add_argument('-f', '--fps', type=int, default=30,
                       help='帧率 (默认：30)')
    parser.add_argument('-d', '--device', type=str, default='/dev/video0',
                       help='摄像头设备 (默认：/dev/video0)')
    parser.add_argument('-p', '--port', type=int, default=8554,
                       help='RTSP 端口 (默认：8554)')
    parser.add_argument('-b', '--bitrate', type=int, default=4000,
                       help='视频比特率 kbps (默认：4000)')
    parser.add_argument('--test', action='store_true',
                       help='使用测试视频源而非摄像头')
    
    return parser.parse_args()

def main():
    args = parse_args()
    
    # 验证参数
    resolution = f"{args.width}x{args.height}"
    print("=" * 60)
    print("  GStreamer RTSP 服务器配置")
    print("=" * 60)
    print(f"  分辨率：   {resolution}")
    print(f"  帧率：     {args.fps} fps")
    print(f"  比特率：   {args.bitrate} kbps")
    print(f"  摄像头：   {args.device}")
    print(f"  端口：     {args.port}")
    if args.test:
        print(f"  视频源：   测试图案 (videotestsrc)")
    else:
        print(f"  视频源：   摄像头 (v4l2src)")
    print("=" * 60)
    sys.stdout.flush()
    
    # 初始化 GStreamer
    Gst.init(sys.argv)
    loop = GLib.MainLoop()
    
    # 创建 RTSP 服务器
    server = GstRtspServer.RTSPServer()
    server.set_service(str(args.port))
    server.set_address('0.0.0.0')
    
    # 构建 GStreamer pipeline
    if args.test:
        # 使用测试视频源
        pipeline = (
            f'( videotestsrc pattern=ball ! '
            f'video/x-raw,width={args.width},height={args.height},framerate={args.fps}/1 ! '
            f'videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency bitrate={args.bitrate} ! '
            f'rtph264pay config-interval=1 name=pay0 pt=96 )'
        )
    else:
        # 使用摄像头（H264 直通，无需重新编码）
        # 注意：摄像头只支持特定分辨率和帧率
        pipeline = (
            f'( v4l2src device={args.device} ! '
            f'video/x-h264,width={args.width},height={args.height},framerate={args.fps}/1 ! '
            f'h264parse ! rtph264pay config-interval=1 name=pay0 pt=96 )'
        )
    
    print(f"\n  GStreamer Pipeline:")
    print(f"  {pipeline}")
    print()
    sys.stdout.flush()
    
    # 创建媒体工厂
    factory = GstRtspServer.RTSPMediaFactory()
    factory.set_launch(pipeline)
    factory.set_shared(True)
    factory.set_buffer_size(0)
    factory.set_latency(0)
    
    # 挂载工厂到路径
    mount_point = server.get_mount_points()
    mount_point.add_factory('/video', factory)
    
    # 附加到主循环
    server.attach(None)
    
    print(f"✓ RTSP 服务器已启动")
    print(f"  访问地址：rtsp://<your-ip>:{args.port}/video")
    print(f"  按 Ctrl+C 停止服务器")
    print("=" * 60)
    sys.stdout.flush()
    
    try:
        loop.run()
    except KeyboardInterrupt:
        print("\n\n正在停止服务器...")
        loop.quit()

if __name__ == "__main__":
    main()
