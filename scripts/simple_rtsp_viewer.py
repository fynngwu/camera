#!/usr/bin/env python3
"""
简单的 RTSP 流查看器 - 易于复用
=================================
用法:
    python3 simple_rtsp_viewer.py
    python3 simple_rtsp_viewer.py --url rtsp://your-ip:8554/video
"""

import cv2
import argparse


def create_pipeline(url: str) -> str:
    """
    创建 GStreamer pipeline 用于接收 RTSP 流

    Args:
        url: RTSP 流地址

    Returns:
        GStreamer pipeline 字符串
    """
    return (
        f"rtspsrc location={url} latency=0 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! "
        "videoconvert ! appsink"
    )


def view_rtsp(url: str, window_name: str = 'RTSP Stream') -> None:
    """
    连接 RTSP 流并显示视频

    Args:
        url: RTSP 流地址
        window_name: 显示窗口名称

    按键操作:
        q / Q: 退出
    """
    # 创建 pipeline 并打开摄像头
    pipeline = create_pipeline(url)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print(f"错误：无法打开 RTSP 流：{url}")
        return

    print(f"已连接：{url}")
    print("按 'q' 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("警告：无法读取帧")
            break

        cv2.imshow(window_name, frame)

        # 按 q 退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 清理资源
    cap.release()
    cv2.destroyAllWindows()
    print("已退出")


def main():
    """主函数 - 解析命令行参数并启动查看器"""
    parser = argparse.ArgumentParser(
        description='简单的 RTSP 流查看器'
    )
    parser.add_argument(
        '-u', '--url',
        type=str,
        default='rtsp://100.103.8.66:8554/video',
        help='RTSP 流地址 (默认：rtsp://100.103.8.66:8554/video)'
    )
    args = parser.parse_args()

    view_rtsp(args.url)


if __name__ == '__main__':
    main()
