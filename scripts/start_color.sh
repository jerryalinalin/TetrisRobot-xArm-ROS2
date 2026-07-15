#!/usr/bin/env bash
set -e

if [ "$#" -lt 1 ] || [ -z "$1" ]; then
    echo "Usage: $0 <robot_ip>" >&2
    exit 2
fi

ROBOT_IP="$1"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$REPO_ROOT/logs"
mkdir -p "$LOG_DIR"

source "$REPO_ROOT/install/setup.bash"

cleanup() {
    echo ""
    echo "正在关闭所有节点..."
    kill $DRIVER_PID $CAM_PID $DETECT_PID 2>/dev/null
    wait 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

# 1. 启动机械臂驱动
echo "[1/4] 启动 xArm6 驱动..."
ros2 launch xarm_api xarm6_driver.launch.py \
    robot_ip:="$ROBOT_IP" add_vacuum_gripper:=true \
    > "$LOG_DIR/driver.log" 2>&1 &
DRIVER_PID=$!

timeout 30 bash -c "until grep -q 'Tcp control connection successful' \"$LOG_DIR/driver.log\" 2>/dev/null; do sleep 0.5; done" || {
    echo "[ERROR] 驱动连接超时"; cleanup; exit 1
}
echo "[OK] 驱动已连接"

# 2. 使能电机
echo "[2/4] 使能电机..."
ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}" > /dev/null 2>&1
sleep 0.3
ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}" > /dev/null 2>&1
sleep 0.3
ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}" > /dev/null 2>&1
echo "[OK] 电机已使能"

# 3. 启动相机
echo "[3/4] 启动 Realsense 相机..."
ros2 launch realsense2_camera rs_launch.py \
    > "$LOG_DIR/camera.log" 2>&1 &
CAM_PID=$!
sleep 3
echo "[OK] 相机已启动"

# 4. 启动颜色识别
echo "[4/4] 启动颜色识别节点..."
ros2 run color_detector detection_node --ros-args \
    -r /camera/color/image_raw:=/camera/camera/color/image_raw \
    > "$LOG_DIR/detection.log" 2>&1 &
DETECT_PID=$!
sleep 1
echo "[OK] 颜色识别节点已启动"
echo ""
echo "按 Ctrl+C 停止。颜色识别日志: logs/detection.log"

# 显示实时输出
tail -f "$LOG_DIR/detection.log"
