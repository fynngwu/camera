#!/bin/bash
# 一键启动 MediaMTX + FFmpeg 推流
# 用法: ./start_rtsp_push.sh
# 停止: ./start_rtsp_push.sh stop

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MTX="$SCRIPT_DIR/mediamtx"
PUSH_URL="rtsp://127.0.0.1:8554/cam"

stop() {
    echo "停止 FFmpeg..."
    pkill -f "ffmpeg.*rtsp.*cam" 2>/dev/null
    echo "停止 MediaMTX..."
    pkill -f "$MTX" 2>/dev/null
    echo "已全部停止"
    exit 0
}

if [ "$1" = "stop" ]; then
    stop
fi

# 清理残留进程
pkill -f "ffmpeg.*rtsp.*cam" 2>/dev/null
pkill -f "$MTX" 2>/dev/null
sleep 1

# 启动 MediaMTX
echo "启动 MediaMTX..."
"$MTX" &
MTX_PID=$!
sleep 2

# 检查 MediaMTX 是否存活
if ! kill -0 $MTX_PID 2>/dev/null; then
    echo "MediaMTX 启动失败"
    exit 1
fi
echo "MediaMTX 已启动 (PID $MTX_PID)"

# 启动 FFmpeg 推流
echo "启动 FFmpeg 推流 -> $PUSH_URL"
ffmpeg \
    -f v4l2 \
    -input_format h264 \
    -framerate 30 \
    -video_size 1920x1080 \
    -i /dev/video0 \
    -c:v copy \
    -f rtsp \
    -rtsp_transport udp \
    -muxdelay 0 \
    "$PUSH_URL" &
FFMPEG_PID=$!
sleep 2

# 检查 FFmpeg 是否存活
if ! kill -0 $FFMPEG_PID 2>/dev/null; then
    echo "FFmpeg 启动失败，停止 MediaMTX"
    kill $MTX_PID 2>/dev/null
    exit 1
fi
echo "FFmpeg 已启动 (PID $FFMPEG_PID)"

echo ""
echo "=== 推流已就绪 ==="
echo "RTSP 地址: rtsp://$(hostname -I | awk '{print $1}'):8554/cam"
echo "停止: $0 stop"
echo "按 Ctrl+C 也可停止全部"
echo ""

# 捕获退出信号，清理进程
trap "echo ''; stop" SIGINT SIGTERM

# 等待任意子进程退出
wait -n $MTX_PID $FFMPEG_PID 2>/dev/null
echo "有进程退出，清理剩余进程..."
stop
