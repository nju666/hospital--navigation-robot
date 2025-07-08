#!/bin/bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash

# 配置参数
MP3_FILE="/root/voice_id/recording.mp3"
LOG_FILE="/root/voice_id/audio_process.log"
RESULT_FILE="/root/voice_id/result.json"
#重启音频文件
# 日志函数
log() {
    local message="[$(date '+%Y-%m-%d %H:%M:%S')] $1"
    echo "$message"
    echo "$message" >> "$LOG_FILE"
}

# 错误处理函数
handle_error() {
    log "❌ 错误: $1"
    exit 1
}

# ✅ 杀死之前的 send_goal_node 节点
log "尝试终止旧的导航节点（send_goal_node）..."
pkill -f send_goal_node || true

# ✅ 后台启动监听（加 & 表示后台运行）
#log "启动目标文件监听服务..."
#python3 /userdata/dev_ws/src/originbot/originbot_send_goal/script/watch_goal_pose.py &
#WATCH_PID=$!

# 开始录音
log "开始录音 (5秒)..."
tinycap "$MP3_FILE" -D 1 -d 1 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5 || handle_error "录音失败"
log "✅ 录音完成: $MP3_FILE"

# 语音转文字
log "开始语音识别..."
speech_text=$(python3 /root/voice_id/mp3_to_text.py --input "$MP3_FILE" 2>> "$LOG_FILE")

if [ -z "$speech_text" ]; then
    handle_error "语音识别失败或结果为空"
fi

log "✅ 识别文本: $speech_text"

# 科室推荐
log "开始科室推荐..."
python3 /root/voice_id/recommendation.py --input "$speech_text"

# 清理
log "清理临时文件..."
rm -f "$MP3_FILE"
log "✅ 清理完成"

# 可选：终止监听进程（如果只监听一次的话）
#kill $WATCH_PID

log "===== 处理完成 ====="
