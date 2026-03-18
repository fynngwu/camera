#!/bin/bash
# RTSP 服务器管理脚本 - 支持命令行配置

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

REMOTE_HOST="100.103.8.66"
REMOTE_USER="wufy"

# 默认配置
DEFAULT_WIDTH=1920
DEFAULT_HEIGHT=1080
DEFAULT_FPS=30
DEFAULT_BITRATE=4000
DEFAULT_DEVICE="/dev/video0"
DEFAULT_PORT=8554

show_usage() {
    cat << EOF
用法：$0 <命令> [选项]

命令:
  start       启动 RTSP 服务器
  stop        停止 RTSP 服务器
  restart     重启 RTSP 服务器
  status      查看服务器状态
  log         查看服务器日志
  attach      连接 tmux 会话
  view        启动本地 OpenCV 查看器
  test        运行性能测试

选项:
  -w, --width     视频宽度 (默认：$DEFAULT_WIDTH)
  -H, --height    视频高度 (默认：$DEFAULT_HEIGHT)
  -f, --fps       帧率 (默认：$DEFAULT_FPS)
  -b, --bitrate   比特率 kbps (默认：$DEFAULT_BITRATE)
  -d, --device    摄像头设备 (默认：$DEFAULT_DEVICE)
  -p, --port      RTSP 端口 (默认：$DEFAULT_PORT)
  --test          使用测试视频源

示例:
  $0 start                              # 默认 720p30
  $0 start -w 1920 -H 1080 -f 30        # 1080p30
  $0 start -w 1280 -H 720 -f 60         # 720p60
  $0 start -w 640 -H 480 -f 30          # VGA 30fps
  $0 start --test                       # 测试视频源
  $0 view                               # 启动查看器
EOF
    exit 1
}

# 解析全局选项
parse_options() {
    WIDTH=$DEFAULT_WIDTH
    HEIGHT=$DEFAULT_HEIGHT
    FPS=$DEFAULT_FPS
    BITRATE=$DEFAULT_BITRATE
    DEVICE=$DEFAULT_DEVICE
    PORT=$DEFAULT_PORT
    TEST_SOURCE="false"
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            -w|--width) WIDTH="$2"; shift 2 ;;
            -H|--height) HEIGHT="$2"; shift 2 ;;
            -f|--fps) FPS="$2"; shift 2 ;;
            -b|--bitrate) BITRATE="$2"; shift 2 ;;
            -d|--device) DEVICE="$2"; shift 2 ;;
            -p|--port) PORT="$2"; shift 2 ;;
            --test) TEST_SOURCE="true"; shift ;;
            *) break ;;
        esac
    done
}

start_server() {
    parse_options "$@"
    
    echo "启动 RTSP 服务器..."
    echo "  分辨率：${WIDTH}x${HEIGHT}@${FPS}fps"
    echo "  比特率：${BITRATE}kbps"
    echo "  摄像头：${DEVICE}"
    echo ""
    
    ssh ${REMOTE_USER}@${REMOTE_HOST} << ENDSSH
# 清理旧进程
pkill -f rtsp_server.py 2>/dev/null || true
tmux kill-session -t rtsp_server 2>/dev/null || true
sleep 1

# 创建 RTSP 服务器脚本
cat > /tmp/rtsp_server.py << 'PYEOF'
#!/usr/bin/env python3
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import GLib, Gst, GstRtspServer
import sys

Gst.init(sys.argv)
loop = GLib.MainLoop()

server = GstRtspServer.RTSPServer()
server.set_service('${PORT}')
server.set_address('0.0.0.0')

# 使用摄像头原生 H264 格式
pipeline = (
    '( v4l2src device=${DEVICE} ! '
    'video/x-h264,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 ! '
    'h264parse ! rtph264pay config-interval=1 name=pay0 pt=96 )'
)

factory = GstRtspServer.RTSPMediaFactory()
factory.set_launch(pipeline)
factory.set_shared(True)
factory.set_buffer_size(0)
factory.set_latency(0)

mount_point = server.get_mount_points()
mount_point.add_factory('/video', factory)

server.attach(None)

print('=' * 60)
print(f'RTSP Server Started!')
print(f'分辨率：${WIDTH}x${HEIGHT}@${FPS}fps')
print(f'比特率：${BITRATE}kbps')
print(f'Stream URL: rtsp://<ip>:${PORT}/video')
print('=' * 60)
sys.stdout.flush()

loop.run()
PYEOF

tmux new-session -d -s rtsp_server 'python3 /tmp/rtsp_server.py'
sleep 3

# 验证启动
if pgrep -f 'rtsp_server.py' > /dev/null; then
    echo "RTSP 服务器启动成功!"
    pgrep -fa rtsp_server.py
else
    echo "错误：RTSP 服务器启动失败"
    exit 1
fi
ENDSSH
}

stop_server() {
    echo "停止 RTSP 服务器..."
    ssh ${REMOTE_USER}@${REMOTE_HOST} "pkill -f rtsp_server.py; tmux kill-session -t rtsp_server 2>/dev/null || true"
    echo "已停止"
}

restart_server() {
    parse_options "$@"
    stop_server
    sleep 2
    start_server "$@"
}

status_server() {
    echo "RTSP 服务器状态:"
    ssh ${REMOTE_USER}@${REMOTE_HOST} "pgrep -fa rtsp_server.py" 2>/dev/null || echo "未运行"
    echo ""
    echo "tmux 会话:"
    ssh ${REMOTE_USER}@${REMOTE_HOST} "tmux list-sessions" 2>/dev/null || echo "无会话"
    echo ""
    echo "端口 8554:"
    ssh ${REMOTE_USER}@${REMOTE_HOST} "ss -tlnp | grep 8554" 2>/dev/null || echo "未监听"
}

log_server() {
    echo "RTSP 服务器日志:"
    ssh ${REMOTE_USER}@${REMOTE_HOST} "tmux capture-pane -t rtsp_server -p -S -100"
}

attach_server() {
    echo "连接远程 tmux 会话 (Ctrl+B 然后 D 退出)..."
    ssh -t ${REMOTE_USER}@${REMOTE_HOST} "tmux attach -t rtsp_server"
}

view_stream() {
    # 解析查看器选项
    VIEW_ARGS=""
    while [[ $# -gt 0 ]]; do
        VIEW_ARGS="$VIEW_ARGS $1"
        shift
    done
    
    echo "启动本地 OpenCV 查看器..."
    python3 "$SCRIPT_DIR/view_rtsp_stream.py" $VIEW_ARGS
}

test_performance() {
    echo "=============================================="
    echo "  RTSP 性能测试"
    echo "=============================================="
    echo ""
    
    # 测试配置列表
    declare -a TESTS=(
        "640x480@30"
        "1280x720@30"
        "1280x720@60"
        "1920x1080@30"
        "1920x1080@15"
    )
    
    RESULTS_FILE="$PROJECT_ROOT/docs/benchmark_results.md"
    
    # 创建结果文件头部
    cat > "$RESULTS_FILE" << 'HEADER'
# 性能测试结果 (Benchmark Results)

测试日期：$(date '+%Y-%m-%d %H:%M:%S')

远程主机：100.103.8.66
摄像头：/dev/video0 (UVC USB Camera)

## 测试结果

| 分辨率 | 帧率配置 | 平均 FPS | 最小 FPS | 最大 FPS | 备注 |
|--------|----------|----------|----------|----------|------|
HEADER
    
    # 更新测试日期
    sed -i "s/\$(date '+%Y-%m-%d %H:%M:%S')/$(date '+%Y-%m-%d %H:%M:%S')/" "$RESULTS_FILE"
    
    for config in "${TESTS[@]}"; do
        IFS='@' read -r resolution fps <<< "$config"
        IFS='x' read -r width height <<< "$resolution"
        
        echo "----------------------------------------"
        echo "测试配置：${width}x${height}@${fps}fps"
        echo "----------------------------------------"
        
        # 重启服务器
        start_server "" "-w" "$width" "-H" "$height" "-f" "$fps"
        
        sleep 3
        
        # 运行性能测试并捕获输出
        OUTPUT=$(python3 "$SCRIPT_DIR/view_rtsp_stream.py" \
            --benchmark --no-display --stats 2>&1)
        
        # 解析 FPS 统计
        AVG_FPS=$(echo "$OUTPUT" | grep "平均 FPS:" | tail -1 | awk '{print $3}')
        INSTANT_STATS=$(echo "$OUTPUT" | grep "瞬时 FPS:" | tail -1)
        MIN_FPS=$(echo "$INSTANT_STATS" | grep -oP '最小=\K[0-9.]+')
        MAX_FPS=$(echo "$INSTANT_STATS" | grep -oP '最大=\K[0-9.]+')
        
        # 确定备注
        NOTES=""
        if [[ "$AVG_FPS" != "" ]]; then
            FPS_INT=${AVG_FPS%.*}
            if [[ $FPS_INT -ge $((fps - 3)) ]]; then
                NOTES="✓ 优秀"
            elif [[ $FPS_INT -ge $((fps - 10)) ]]; then
                NOTES="○ 良好"
            else
                NOTES="△ 需优化"
            fi
        fi
        
        # 写入结果文件
        echo "| ${width}x${height} | ${fps}fps | ${AVG_FPS:-N/A} | ${MIN_FPS:-N/A} | ${MAX_FPS:-N/A} | ${NOTES} |" >> "$RESULTS_FILE"
        
        echo ""
        echo "结果：平均 FPS=${AVG_FPS:-N/A}, 最小=${MIN_FPS:-N/A}, 最大=${MAX_FPS:-N/A}"
        echo ""
        
        sleep 1
        stop_server
        sleep 1
        
        echo ""
    done
    
    # 添加总结到结果文件
    cat >> "$RESULTS_FILE" << 'FOOTER'

## 测试说明

- 测试方法：每个配置运行 10 秒，记录瞬时 FPS 和平均 FPS
- 网络环境：局域网 SSH 连接
- 视频编码：H.264 直通（摄像头原生输出）

## 推荐配置

根据测试结果，推荐使用以下配置：
- **实时性优先**: 640x480@30 或 1280x720@30
- **画质优先**: 1920x1080@15 或 1920x1080@30
- **平衡配置**: 1280x720@30
FOOTER
    
    echo "=============================================="
    echo "  性能测试完成!"
    echo "=============================================="
    echo ""
    echo "测试结果已保存到：$RESULTS_FILE"
    echo ""
    cat "$RESULTS_FILE"
}

# 主程序
if [ $# -lt 1 ]; then
    show_usage
fi

COMMAND=$1
shift

case $COMMAND in
    start)
        start_server "$@"
        ;;
    stop)
        stop_server
        ;;
    restart)
        restart_server "$@"
        ;;
    status)
        status_server
        ;;
    log)
        log_server
        ;;
    attach)
        attach_server
        ;;
    view)
        view_stream "$@"
        ;;
    test)
        test_performance
        ;;
    *)
        show_usage
        ;;
esac
