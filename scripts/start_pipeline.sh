#!/usr/bin/env bash
set -e

if [ "$#" -lt 1 ] || [ -z "$1" ]; then
    echo "Usage: $0 <robot_ip> [workcell_config]" >&2
    exit 2
fi

ROBOT_IP="$1"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${2:-$REPO_ROOT/config/workcell.example.yaml}"
LOG_DIR="$REPO_ROOT/logs"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "[ERROR] Workcell config not found: $CONFIG_FILE" >&2
    exit 2
fi
mkdir -p "$LOG_DIR"

source "$REPO_ROOT/install/setup.bash"

cleanup() {
    echo ""
    echo "正在关闭所有节点..."
    kill $DRIVER_PID $CAM_PID $MOVE_PID $DETECT_PID $SORTER_PID 2>/dev/null
    wait 2>/dev/null
    echo "已退出。"
    exit 0
}
trap cleanup SIGINT SIGTERM

print_header() {
    echo ""
    echo "============================================"
    echo "  $1"
    echo "============================================"
}

# 1. xArm6 驱动
print_header "[1/5] xArm6 驱动"
ros2 launch xarm_api xarm6_driver.launch.py \
    robot_ip:="$ROBOT_IP" add_vacuum_gripper:=true \
    > "$LOG_DIR/driver.log" 2>&1 &
DRIVER_PID=$!

echo "等待驱动连接..."
if ! timeout 30 bash -c "until grep -q 'Tcp control connection successful' \"$LOG_DIR/driver.log\" 2>/dev/null; do sleep 0.5; done"; then
    echo "[ERROR] 驱动连接超时，请检查机械臂 IP 和网络"
    cleanup; exit 1
fi
echo "[OK] 驱动已连接"
sleep 1

# 2. 使能电机
print_header "[2/5] 使能电机"
ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}" > /dev/null 2>&1
sleep 0.3
ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}" > /dev/null 2>&1
sleep 0.3
ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}" > /dev/null 2>&1
echo "[OK] 电机已使能"

# 3. 启动 Realsense 相机
print_header "[3/5] Realsense 相机"
ros2 launch realsense2_camera rs_launch.py \
    > "$LOG_DIR/camera.log" 2>&1 &
CAM_PID=$!
sleep 4
echo "[OK] 相机已启动"

# 4. MoveOnce 抓取服务
print_header "[4/5] MoveOnce 抓取服务"
ros2 run block_sorter move_service --ros-args --params-file "$CONFIG_FILE" \
    > "$LOG_DIR/move_service.log" 2>&1 &
MOVE_PID=$!

echo "等待 move_service 就绪..."
if ! timeout 15 bash -c "until grep -q 'MoveOnce service ready' \"$LOG_DIR/move_service.log\" 2>/dev/null; do sleep 0.5; done"; then
    echo "[ERROR] move_service 启动超时"
    cleanup; exit 1
fi
echo "[OK] MoveOnce 就绪"
echo "等待归零完成..."
sleep 5

# 5. 颜色识别 + 策略调度
print_header "[5/5] 颜色识别 + 策略调度"

ros2 run color_detector detection_node --ros-args \
    -r /camera/color/image_raw:=/camera/camera/color/image_raw \
    > "$LOG_DIR/detection.log" 2>&1 &
DETECT_PID=$!
sleep 2

ros2 run block_sorter sorter_node --ros-args --params-file "$CONFIG_FILE" \
    > "$LOG_DIR/sorter.log" 2>&1 &
SORTER_PID=$!

echo ""
echo "============================================"
echo "  全流程已启动！日志: logs/"
echo "  放一个方块在工作台上测试"
echo "  按 Ctrl+C 停止所有节点"
echo "============================================"
echo ""

# 实时显示检测 + 策略调度日志
tail -f "$LOG_DIR/detection.log" "$LOG_DIR/sorter.log"
