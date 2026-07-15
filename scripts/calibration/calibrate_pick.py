#!/usr/bin/env python3
"""
取料标定工具 — 点击相机画面9个点 + 键盘控制机械臂对准 → 自动算矩阵

标定板: A4纸上的3×3网格（点间距请在纸上量好后填入）

步骤:
  1. 按 S 保存当前相机画面
  2. 在画面上依次点击 9 个点（或拖动滑块自动生成网格点）
  3. 键盘控制机械臂对准每个物理点，按空格记录
  4. 自动计算 transform_mat
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import MoveCartesian
import cv2
import numpy as np
import os
import sys
import termios
import tty
import select
import time
import json

# =============== 配置 ===============
GRID_ROWS = 3
GRID_COLS = 3
STEP_LINEAR = 2.0  # mm
STEP_ANGLE = 0.02  # rad
MOVE_SPEED = 100.0
MOVE_ACC = 500.0
# ===================================

POINTS_TOTAL = GRID_ROWS * GRID_COLS

# 网格点在标定板上的物理间距（单位: mm），仅供参考
# 实际计算用的是点击的像素坐标 + 记录的机器人坐标
GRID_SPACING_X = 60  # 列间距
GRID_SPACING_Y = 60  # 行间距


class PickCalibrator(Node):
    def __init__(self):
        super().__init__('pick_calibrator')
        self.bridge = CvBridge()
        self.latest_frame = None

        # ROS2 接口
        self.img_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.img_callback, 1)
        self.pose_sub = self.create_subscription(
            RobotMsg, '/xarm/robot_states', self.pose_callback, 10)
        self.move_client = self.create_client(
            MoveCartesian, '/xarm/set_position')
        self.move_client.wait_for_service(timeout_sec=10.0)

        self.current_pose = None
        self.pose_ready = False

        # 标定数据
        self.pixel_points = []     # [(u1,v1), ...] 9个像素点
        self.robot_points = []     # [(x1,y1), ...] 9个机器人坐标
        self.saved_image = None    # 用于点击的图片
        self.click_mode = False    # 是否处于点击模式

        self.get_logger().info('取料标定工具已启动，等待相机画面...')

    def img_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def pose_callback(self, msg):
        self.current_pose = list(msg.pose)
        self.pose_ready = True

    def move_relative(self, dx=0, dy=0, dz=0, dyaw=0):
        if self.current_pose is None:
            return
        pose = self.current_pose.copy()
        pose[0] += dx
        pose[1] += dy
        pose[2] += dz
        pose[5] += dyaw
        req = MoveCartesian.Request()
        req.pose = [float(v) for v in pose]
        req.speed = MOVE_SPEED
        req.acc = MOVE_ACC
        req.radius = 20.0
        req.wait = False
        req.mvtime = 0.0
        self.move_client.call_async(req)


def mouse_callback(event, x, y, flags, param):
    """鼠标点击回调"""
    data = param
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(data['pixel_points']) < POINTS_TOTAL:
            data['pixel_points'].append((x, y))
            idx = len(data['pixel_points'])
            print(f'  ✅ 已标记点 {idx}/{POINTS_TOTAL}: 像素({x}, {y})')
            # 画点
            cv2.circle(data['disp_img'], (x, y), 6, (0, 255, 0), -1)
            cv2.putText(data['disp_img'], str(idx), (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow('Calibration - Click 9 points', data['disp_img'])
            cv2.waitKey(1)
        if len(data['pixel_points']) == POINTS_TOTAL:
            print(f'\n  ✅ 9 个像素点已全部标记')
            print(f'  关闭图片窗口后，用键盘控制机械臂依次对准每个点并记录')


def get_key(timeout=0.05):
    """非阻塞键盘读取"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        dr, _, _ = select.select([fd], [], [], timeout)
        if dr:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_help_jog():
    print("""
  ┌───────────────────────────────────────────────┐
  │  机械臂控制                                    │
  ├───────────────────────────────────────────────┤
  │  W/S     X+ / X-        R/F    Yaw+ / -      │
  │  A/D     Y- / Y+        Z     Z轴微调模式    │
  │  Q/E     Z+ / Z-        +/-   步长调整        │
  │  Space   记录当前点为机器人坐标                │
  │  H       帮助    Esc/Ctrl+C 退出              │
  └───────────────────────────────────────────────┘
  """)


def main():
    rclpy.init()
    node = PickCalibrator()

    # 等待相机画面
    print('等待相机画面...')
    timeout = 15.0
    start = time.time()
    while node.latest_frame is None and time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.latest_frame is None:
        print('[ERROR] 未收到相机画面，请确保相机已启动')
        print('  ros2 launch realsense2_camera rs_launch.py')
        node.destroy_node()
        rclpy.shutdown()
        return
    print(f'[OK] 收到相机画面 ({node.latest_frame.shape[1]}x{node.latest_frame.shape[0]})')

    if node.pose_ready:
        print(f'[OK] 机械臂位姿: {[f"{v:.1f}" for v in node.current_pose]}')
    else:
        print('[INFO] 机械臂未连接，Phase 3 前需启动驱动')

    # ========== Phase 1: 抓取相机画面 ==========
    print('\n' + '='*60)
    print(' Phase 1: 拍摄标定板画面')
    print('='*60)
    print(' 把 A4 纸（带 9 个点）放在工作台上相机视野内')
    print(' 按 S 保存当前相机画面')
    print(' 按任意其他键重新等待画面更新，直到满意')
    print(' 按 Q 退出')

    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest_frame is not None:
            display = node.latest_frame.copy()
            cv2.imshow('Camera Preview', display)
            cv2.waitKey(1)

        key = get_key(0.1)
        if key is None:
            continue
        if key in ('s', 'S'):
            node.saved_image = node.latest_frame.copy()
            print(f'  ✅ 画面已保存 ({node.saved_image.shape[1]}x{node.saved_image.shape[0]})')
            cv2.destroyWindow('Camera Preview')
            break
        elif key in ('q', 'Q', '\x03'):
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()
            return

    # ========== Phase 2: 点击像素点 ==========
    print('\n' + '='*60)
    print(' Phase 2: 标记 9 个点的像素坐标')
    print('='*60)
    print(' 在弹出窗口中，从左到右、从上到下依次点击 9 个点:')
    print(f'   行1: (0,0) → ({GRID_COLS-1},0)')
    print(f'   行2: (0,1) → ({GRID_COLS-1},1)')
    print(f'   行3: (0,2) → ({GRID_COLS-1},2)')
    print(' 关掉图片窗口后进入下一步\n')

    data = {'pixel_points': node.pixel_points,
            'disp_img': node.saved_image.copy()}
    cv2.namedWindow('Calibration - Click 9 points')
    cv2.setMouseCallback('Calibration - Click 9 points', mouse_callback, data)

    # 显示图片并等待点击
    disp = node.saved_image.copy()
    cv2.putText(disp, 'Click the 9 points (top-left to bottom-right)',
                (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow('Calibration - Click 9 points', disp)
    cv2.waitKey(1)

    # 等待用户点完并关窗
    while len(node.pixel_points) < POINTS_TOTAL:
        cv2.waitKey(100)
        rclpy.spin_once(node, timeout_sec=0.01)

    # 等待用户关闭窗口
    print(' 请关闭图片窗口，然后进入 Phase 3...')
    while cv2.getWindowProperty('Calibration - Click 9 points', cv2.WND_PROP_VISIBLE) > 0:
        cv2.waitKey(100)
    cv2.destroyAllWindows()
    cv2.waitKey(100)

    # ========== Phase 3: 记录机器人坐标 ==========
    print('\n' + '='*60)
    print(' Phase 3: 记录 9 个点的机器人坐标')
    print('='*60)
    print_help_jog()

    step_linear = STEP_LINEAR
    step_angle = STEP_ANGLE
    fine_mode = False
    corner_idx = 0
    first_yaw = None
    running = True

    try:
        while running and rclpy.ok() and corner_idx < POINTS_TOTAL:
            rclpy.spin_once(node, timeout_sec=0.02)

            # 显示当前要去的点
            pu, pv = node.pixel_points[corner_idx]
            status = (f'\r  点 {corner_idx+1}/{POINTS_TOTAL}: '
                      f'像素({pu},{pv})  '
                      f'当前位姿: ({node.current_pose[0]:.1f}, {node.current_pose[1]:.1f}, '
                      f'{node.current_pose[2]:.1f}) yaw={node.current_pose[5]:.2f}  '
                      f'步长: {step_linear:.1f}mm')
            print(status + '   ' * 5, end='', flush=True)

            key = get_key(0.05)
            if key is None:
                continue

            dx = dy = dz = dyaw = 0.0

            if key in ('w', 'W'):
                dx = step_linear
            elif key in ('s', 'S'):
                dx = -step_linear
            elif key in ('a', 'A'):
                dy = -step_linear
            elif key in ('d', 'D'):
                dy = step_linear
            elif key in ('q', 'Q'):
                dz = step_linear
            elif key in ('e', 'E'):
                dz = -step_linear
            elif key in ('r', 'R'):
                dyaw = step_angle
            elif key in ('f', 'F'):
                dyaw = -step_angle
            elif key == ' ':
                # 记录
                rx = node.current_pose[0]
                ry = node.current_pose[1]
                yaw = node.current_pose[5]
                node.robot_points.append((rx, ry))
                if first_yaw is None:
                    first_yaw = yaw
                elif abs(yaw - first_yaw) > 0.05:
                    print(f'\n  ⚠️  Yaw 偏差: {yaw:.2f} vs 首次 {first_yaw:.2f}')
                print(f'\n  ✅ 已记录点 {corner_idx+1}/{POINTS_TOTAL}: '
                      f'机器人({rx:.2f}, {ry:.2f})')
                corner_idx += 1
                continue
            elif key == 'z':
                fine_mode = not fine_mode
                step_linear = max(0.5, step_linear / 2) if fine_mode else STEP_LINEAR
                step_angle = max(0.005, step_angle / 2) if fine_mode else STEP_ANGLE
                print(f'\n  {"微调" if fine_mode else "普通"}模式: '
                      f'步长 {step_linear:.1f}mm / {step_angle:.3f}rad')
                continue
            elif key in ('+', '='):
                step_linear = min(50.0, step_linear * 2)
                step_angle = min(0.2, step_angle * 2)
            elif key in ('-', '_'):
                step_linear = max(0.1, step_linear / 2)
                step_angle = max(0.001, step_angle / 2)
            elif key in ('h', 'H'):
                print()
                print_help_jog()
                continue
            elif key in ('\x03', '\x1b'):
                running = False
                break
            else:
                continue

            if any([dx, dy, dz, dyaw]):
                node.move_relative(dx, dy, dz, dyaw)

    except KeyboardInterrupt:
        pass

    print()
    if len(node.robot_points) < 3:
        print('[ERROR] 至少需要 3 个点')
        node.destroy_node()
        rclpy.shutdown()
        return

    # ========== Phase 4: 计算矩阵 ==========
    print('\n' + '='*60)
    print(' Phase 4: 计算变换矩阵')
    print('='*60)

    src_pts = np.array(node.pixel_points, dtype=np.float32)
    dst_pts = np.array(node.robot_points, dtype=np.float32)

    print(f'\n  像素坐标: {src_pts.tolist()}')
    print(f'  机器人坐标: {dst_pts.tolist()}')

    # 用 OpenCV 估计仿射变换
    M, inliers = cv2.estimateAffine2D(src_pts, dst_pts)

    if M is None:
        print('[ERROR] 仿射变换计算失败')
    else:
        print(f'\n  变换矩阵 (transform_mat):')
        print(f'  {M[0,0]:.16f}, {M[0,1]:.16f}, {M[0,2]:.16f}')
        print(f'  {M[1,0]:.16f}, {M[1,1]:.16f}, {M[1,2]:.16f}')

        print('\n' + '='*60)
        print('  更新到 config/workcell.example.yaml 的参数:')
        print('='*60)
        print(f'\npick_transform: [{M[0,0]:.16f}, {M[0,1]:.16f}, {M[0,2]:.16f},')
        print(f'                 {M[1,0]:.16f}, {M[1,1]:.16f}, {M[1,2]:.16f}]')

        # 验证误差
        print(f'\n  验证误差:')
        print(f'  {"点":<6} {"像素":<20} {"实际机器人":<22} {"计算机器人":<22} {"误差(mm)":<12}')
        pred = (M @ np.column_stack([src_pts, np.ones(len(src_pts))]).T).T
        for i, ((pu, pv), (rx, ry), (px, py)) in enumerate(
                zip(src_pts, dst_pts, pred)):
            err = np.sqrt((rx - px)**2 + (ry - py)**2)
            print(f'  {i+1:<5} ({pu:<4},{pv:<4})     ({rx:<8.2f},{ry:<8.2f})  '
                  f'({px:<8.2f},{py:<8.2f})  {err:<.3f}')

        avg_err = np.mean([
            np.sqrt((rx - px)**2 + (ry - py)**2)
            for (rx, ry), (px, py) in zip(dst_pts, pred)
        ])
        print(f'\n  平均误差: {avg_err:.3f} mm')

        # 保存到文件
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
        calib_file = os.path.join(repo_root, 'config', 'calib_result.txt')
        with open(calib_file, 'w') as f:
            f.write('transform_mat:\n')
            f.write(f'{M[0,0]:.16f}, {M[0,1]:.16f}, {M[0,2]:.16f}\n')
            f.write(f'{M[1,0]:.16f}, {M[1,1]:.16f}, {M[1,2]:.16f}\n\n')
            f.write(f'avg_error: {avg_err:.3f} mm\n')
            f.write(f'pixel_points: {src_pts.tolist()}\n')
            f.write(f'robot_points: {dst_pts.tolist()}\n')
        print(f'\n  结果已保存到: {calib_file}')

    # 清理
    cv2.destroyAllWindows()
    cv2.waitKey(100)
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN,
                      termios.tcgetattr(sys.stdin.fileno()))
    node.destroy_node()
    rclpy.shutdown()
    print('\n退出。')


if __name__ == '__main__':
    main()
