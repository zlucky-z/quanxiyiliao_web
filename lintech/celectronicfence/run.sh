#!/bin/bash
set -euo pipefail

detect_stream_root() {
    local candidate
    for candidate in \
        /data/sophon-stream-master \
        /data/sophon-stream \
        /data/mediacal/sophon-stream-master \
        /data/mediacal/sophon-stream; do
        if [ -d "$candidate" ]; then
            echo "$candidate"
            return 0
        fi
    done
    echo "/data/sophon-stream-master"
}

run_with_privilege() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    else
        sudo "$@"
    fi
}

# 仅停止 run_hdmi_show.sh 拉起的默认 yolov8_demo，不误杀其他任务 main 进程
terminate_program() {
    local program_pattern='main --demo_config_path=.*yolov8_demo\.json([[:space:]]|$)'
    mapfile -t pids < <(pgrep -f "$program_pattern" 2>/dev/null || true)
    if [ "${#pids[@]}" -eq 0 ]; then
        echo "未发现默认 yolov8_demo 进程。"
        return
    fi

    echo "检测到默认 yolov8_demo 进程: ${pids[*]}，开始停止..."
    local pid
    for pid in "${pids[@]}"; do
        run_with_privilege kill -15 "$pid" 2>/dev/null || true
    done

    local wait_counter=0
    while [ "$wait_counter" -lt 10 ]; do
        local alive=0
        for pid in "${pids[@]}"; do
            if ps -p "$pid" >/dev/null 2>&1; then
                alive=1
                break
            fi
        done
        [ "$alive" -eq 0 ] && return
        wait_counter=$((wait_counter + 1))
        sleep 0.5
    done

    echo "仍有进程未退出，执行强制终止..."
    for pid in "${pids[@]}"; do
        run_with_privilege kill -9 "$pid" 2>/dev/null || true
    done
}

terminate_program

stream_root="$(detect_stream_root)"
script_dir="${stream_root}/samples/yolov8/scripts"
if [ ! -d "$script_dir" ]; then
    echo "错误：脚本目录不存在: ${script_dir}"
    exit 1
fi

echo "启动新脚本: ${script_dir}/run_hdmi_show.sh"
cd "$script_dir"
run_with_privilege ./run_hdmi_show.sh
