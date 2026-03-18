#!/usr/bin/env python3
"""
OpenCV RTSP 查看器 - 支持命令行配置和帧率统计
用法：
    python3 view_rtsp_stream.py                           # 默认连接
    python3 view_rtsp_stream.py --url rtsp://...          # 自定义 URL
    python3 view_rtsp_stream.py --stats                   # 显示详细统计
    python3 view_rtsp_stream.py --record                  # 录制视频
"""

import cv2
import sys
import argparse
import datetime
import os

def parse_args():
    parser = argparse.ArgumentParser(
        description='OpenCV RTSP 流查看器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                                      # 默认连接
  %(prog)s -u rtsp://100.103.8.66:8554/video   # 自定义 URL
  %(prog)s --stats                              # 显示详细统计
  %(prog)s -r output.avi                        # 录制视频
  %(prog)s --no-display --benchmark             # 性能测试（不显示）
        """
    )
    
    parser.add_argument('-u', '--url', type=str, 
                       default='rtsp://100.103.8.66:8554/video',
                       help='RTSP 流地址 (默认：rtsp://100.103.8.66:8554/video)')
    parser.add_argument('--stats', action='store_true',
                       help='显示详细统计信息')
    parser.add_argument('-r', '--record', type=str, metavar='FILE',
                       help='录制视频到文件')
    parser.add_argument('--no-display', action='store_true',
                       help='不显示窗口（用于性能测试）')
    parser.add_argument('--benchmark', action='store_true',
                       help='运行性能测试（10 秒后自动退出）')
    parser.add_argument('--timeout', type=int, default=0,
                       help='自动退出超时（秒，0=无限）')
    
    return parser.parse_args()

def create_pipeline(url):
    """创建 GStreamer pipeline"""
    return (
        f"rtspsrc location={url} latency=0 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! "
        "videoconvert ! appsink"
    )

def main():
    args = parse_args()
    
    print("=" * 60)
    print("  OpenCV RTSP 查看器")
    print("=" * 60)
    print(f"  RTSP 地址：  {args.url}")
    print(f"  详细统计：  {'是' if args.stats else '否'}")
    if args.record:
        print(f"  录制文件：  {args.record}")
    if args.no_display:
        print(f"  显示模式：  禁用（性能测试）")
    if args.benchmark:
        print(f"  性能测试：  启用（10 秒）")
    print("=" * 60)
    sys.stdout.flush()
    
    # 创建 GStreamer pipeline
    pipeline = create_pipeline(args.url)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("错误：无法打开 RTSP 流")
        print("请确保远程主机的 RTSP 服务器已启动")
        sys.exit(1)
    
    print("✓ 成功连接 RTSP 流")
    print("  操作：Q-退出 | S-截图 | P-暂停 | H-帮助")
    print("=" * 60)
    
    # 初始化变量
    paused = False
    frame_count = 0
    saved_count = 0
    start_time = datetime.datetime.now()
    last_stats_time = start_time
    fps_history = []
    frame_times = []
    
    # 录制初始化
    writer = None
    if args.record:
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        # 稍后根据实际帧大小初始化
    
    # 帮助信息
    def show_help():
        print("\n  快捷键:")
        print("    Q - 退出")
        print("    S - 截图保存")
        print("    P - 暂停/继续")
        print("    H - 显示帮助")
        print("    R - 重置统计")
        print()
    
    benchmark_done = False
    
    while True:
        current_time = datetime.datetime.now()
        
        if not paused:
            ret, frame = cap.read()
            
            if not ret:
                print("警告：无法读取帧，尝试重新连接...")
                cap.release()
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                continue
            
            frame_count += 1
            
            # 记录帧时间用于计算瞬时 FPS
            frame_times.append(current_time)
            # 保留最近 1 秒的帧
            cutoff = current_time - datetime.timedelta(seconds=1)
            frame_times = [t for t in frame_times if t > cutoff]
            
            # 计算统计信息
            elapsed = (current_time - start_time).total_seconds()
            avg_fps = frame_count / elapsed if elapsed > 0 else 0
            instant_fps = len(frame_times)
            fps_history.append(instant_fps)
            
            # 每秒打印统计
            stats_elapsed = (current_time - last_stats_time).total_seconds()
            if stats_elapsed >= 1.0 and args.stats:
                print(f"  [统计] 平均 FPS: {avg_fps:.1f} | 瞬时 FPS: {instant_fps} | 总帧数：{frame_count}")
                last_stats_time = current_time
            
            # 录制
            if writer is None and args.record:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(args.record, fourcc, 30.0, (w, h))
                print(f"  开始录制：{args.record} ({w}x{h})")
            if writer:
                writer.write(frame)
            
            # 显示（如果启用）
            if not args.no_display:
                # 在帧上显示信息
                info_text = f"FPS: {instant_fps} | Avg: {avg_fps:.1f} | Frames: {frame_count}"
                cv2.putText(frame, info_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow('RTSP Stream - Q:quit, S:screenshot, P:pause, H:help', frame)
        
        # 按键处理
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q'):
            print("\n退出播放")
            break
        elif key == ord('s') or key == ord('S'):
            filename = f"screenshot_{saved_count:03d}.png"
            cv2.imwrite(filename, frame)
            print(f"截图已保存：{filename}")
            saved_count += 1
        elif key == ord('p') or key == ord('P'):
            paused = not paused
            if paused:
                print("已暂停")
            else:
                print("继续播放")
        elif key == ord('h') or key == ord('H'):
            show_help()
        elif key == ord('r') or key == ord('R'):
            print(f"统计已重置")
            frame_count = 0
            start_time = current_time
            fps_history = []
        
        # 性能测试自动退出
        if args.benchmark:
            elapsed = (current_time - start_time).total_seconds()
            if elapsed >= 10 and not benchmark_done:
                benchmark_done = True
                break
        
        # 超时退出
        if args.timeout > 0:
            elapsed = (current_time - start_time).total_seconds()
            if elapsed >= args.timeout:
                print(f"\n达到超时 ({args.timeout}秒)")
                break
    
    # 清理
    if writer:
        writer.release()
        print(f"录制已保存：{args.record}")
    
    cap.release()
    if not args.no_display:
        cv2.destroyAllWindows()
    
    # 最终统计
    elapsed = (current_time - start_time).total_seconds()
    print("\n" + "=" * 60)
    print("  最终统计")
    print("=" * 60)
    print(f"  总帧数：    {frame_count}")
    print(f"  总时长：    {elapsed:.1f}秒")
    print(f"  平均 FPS:   {frame_count/elapsed:.1f}" if elapsed > 0 else "  平均 FPS:   N/A")
    
    if fps_history:
        avg_instant = sum(fps_history) / len(fps_history)
        min_fps = min(fps_history)
        max_fps = max(fps_history)
        print(f"  瞬时 FPS:   平均={avg_instant:.1f}, 最小={min_fps}, 最大={max_fps}")
    
    if args.record:
        print(f"  录制文件：  {args.record}")
    print("=" * 60)

if __name__ == "__main__":
    main()
