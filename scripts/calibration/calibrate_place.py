#!/usr/bin/env python3
"""
放料盘标定工具 — 手动移动机械臂到 4 个角点，计算变换矩阵

使用方法:
  1. 确保 xArm6 驱动已启动、电机已使能
  2. 机械臂末端夹持器尖端对准放料盘的第 1 个角点，按 R 记录
  3. 依次移动到第 2,3,4 个角点，按 R 记录
  4. 按 P 打印变换矩阵，更新 config/workcell.example.yaml

按键说明:
  W/S      X+ / X- (前后)
  A/D      Y- / Y+ (左右)
  Q/E      Z+ / Z- (上下)
  R/F      Yaw+ / Yaw- (末端旋转)
  T/G      Roll+ / Roll- (俯仰调节)
  Z       切换到 Z 轴微调模式 (步长减半)
  Space    记录当前位姿为角点
  P        打印所有已记录的点 + 计算变换矩阵
  +/-      增大/减小步长
  H        帮助
  Esc/Ctrl+C 退出

放料盘网格: 14行 (row=0~13) x 10列 (col=0~9)
  记录顺序: 左上(0,0) → 左下(13,0) → 右下(13,9) → 右上(0,9)

  重要: 记录全部 4 个角点时，请保持末端 Yaw 朝向一致（用 R/F 调整），
       否则 TCP 工具偏移会导致坐标偏差。
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import MoveCartesian
import sys
import os
import termios
import tty
import select
import time
import numpy as np

# =============== 配置 ===============
GRID_ROWS = 14
GRID_COLS = 10
STEP_LINEAR = 2.0        # mm
STEP_ANGLE = 0.02        # rad
MOVE_SPEED = 100.0       # mm/s
MOVE_ACC = 500.0
# ===================================

# 网格四个角点坐标（按用户记录顺序: 左上→左下→右下→右上）
CORNERS = [
    (0, 0),       # Corner 1: 左上
    (GRID_ROWS - 1, 0),   # Corner 2: 左下
    (GRID_ROWS - 1, GRID_COLS - 1),  # Corner 3: 右下
    (0, GRID_COLS - 1),  # Corner 4: 右上
]


class CalibratorNode(Node):
    def __init__(self):
        super().__init__('calibrator')
        self.pose_sub = self.create_subscription(
            RobotMsg, '/xarm/robot_states', self.pose_callback, 10)
        self.move_client = self.create_client(
            MoveCartesian, '/xarm/set_position')

        self.current_pose = None  # [x, y, z, roll, pitch, yaw]
        self.pose_ready = False
        self.recorded = []  # list of (grid_row, grid_col, robot_x, robot_y, yaw)
        self.first_yaw = None  # Track yaw consistency

        # Wait for services
        self.get_logger().info('等待 /xarm/set_position 服务...')
        self.move_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('服务已就绪')

    def pose_callback(self, msg):
        self.current_pose = list(msg.pose)
        self.pose_ready = True

    def wait_for_pose(self, timeout=3.0):
        start = time.time()
        while not self.pose_ready and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose_ready

    def move_relative(self, dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0):
        if self.current_pose is None:
            self.get_logger().warn('当前位姿未知，忽略移动')
            return
        pose = self.current_pose.copy()
        pose[0] += dx
        pose[1] += dy
        pose[2] += dz
        pose[3] += droll
        pose[4] += dpitch
        pose[5] += dyaw

        req = MoveCartesian.Request()
        req.pose = [float(v) for v in pose]
        req.speed = MOVE_SPEED
        req.acc = MOVE_ACC
        req.radius = 20.0
        req.wait = False
        req.mvtime = 0.0
        future = self.move_client.call_async(req)
        return future

    def record_current(self, corner_idx):
        if self.current_pose is None:
            self.get_logger().error('当前位姿未知，无法记录')
            return False
        grid_r, grid_c = CORNERS[corner_idx]
        rx = self.current_pose[0]
        ry = self.current_pose[1]
        yaw = self.current_pose[5]
        self.recorded.append((grid_r, grid_c, rx, ry, yaw))

        # Check yaw consistency
        if self.first_yaw is None:
            self.first_yaw = yaw
        elif abs(yaw - self.first_yaw) > 0.05:
            print(f'\n  ⚠️  警告: 当前 yaw={yaw:.2f} 与首次记录 yaw={self.first_yaw:.2f} 不一致！')
            print(f'      建议用 R/F 调整回 {self.first_yaw:.2f} 再记录，以免工具偏移影响精度')

        print(f'\n  ✅ 已记录角点 {corner_idx+1}: 网格({grid_r},{grid_c}) → '
              f'机器人({rx:.2f}, {ry:.2f}), yaw={yaw:.2f}')
        return True

    def compute_transform(self):
        if len(self.recorded) < 3:
            print('\n  ❌ 至少需要记录 3 个角点才能计算变换矩阵')
            return None

        # 构建最小二乘问题:
        #   robot_x = a11 * grid_r + a12 * grid_c + a13
        #   robot_y = a21 * grid_r + a22 * grid_c + a23
        A = []
        bx = []
        by = []
        for gr, gc, rx, ry, _ in self.recorded:
            A.append([gr, gc, 1.0])
            bx.append(rx)
            by.append(ry)

        A = np.array(A)
        bx = np.array(bx)
        by = np.array(by)

        # 最小二乘解
        mx, _, _, _ = np.linalg.lstsq(A, bx, rcond=None)
        my, _, _, _ = np.linalg.lstsq(A, by, rcond=None)

        M = np.zeros((2, 3))
        M[0] = mx
        M[1] = my
        return M

    def print_transform_code(self, M):
        print('\n' + '='*70)
        print('  计算得到的 place_transform，更新到 config/workcell.example.yaml:')
        print('='*70)
        print(f'\nplace_transform: [{M[0,0]:.16f}, {M[0,1]:.16f}, {M[0,2]:.16f},')
        print(f'                  {M[1,0]:.16f}, {M[1,1]:.16f}, {M[1,2]:.16f}]')
        print()

        # Verification: for each recorded point, show the error
        print('  验证误差:')
        print(f'  {"角点":<10} {"实际 x":<12} {"计算 x":<12} {"误差 x":<12} {"实际 y":<12} {"计算 y":<12} {"误差 y":<12}  yaw')
        for gr, gc, rx, ry, yaw in self.recorded:
            pred = M @ np.array([gr, gc, 1.0])
            ex = rx - pred[0]
            ey = ry - pred[1]
            print(f'  ({gr},{gc})       {rx:<12.2f} {pred[0]:<12.2f} {ex:<12.4f} '
                  f'{ry:<12.2f} {pred[1]:<12.2f} {ey:<12.4f}  {yaw:.2f}')

        # Check unused corners
        recorded_set = set((gr, gc) for gr, gc, _, _, _ in self.recorded)
        for gr, gc in CORNERS:
            if (gr, gc) not in recorded_set:
                print(f'  ⚠ 角点 ({gr},{gc}) 未记录，建议补充以提高精度')

        print()


def get_key_blocking(timeout=0.05):
    """Non-blocking key read, returns None if no key pressed within timeout."""
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


def print_help():
    print("""
  ┌────────────────────────────────────────────┐
  │  放料盘标定 — 按键控制                      │
  ├────────────────────────────────────────────┤
  │  W/S     X+ / X-       T/G    Roll+ / -    │
  │  A/D     Y- / Y+       R/F    Yaw+ / -     │
  │  Q/E     Z+ / Z-       Z     Z轴微调模式   │
  │  Space   记录当前角点                       │
  │  P       打印结果                           │
  │  +/-     步长:增大/减小                     │
  │  H       帮助    Esc/Ctrl+C 退出            │
  └────────────────────────────────────────────┘
  角点顺序: 左上(0,0) → 左下(13,0) → 右下(13,9) → 右上(0,9)
  保持所有角点 Yaw 一致（用 R/F 调整到相同角度再记录）
  当前步长: {:.1f}mm / {:.3f}rad
    """.format(STEP_LINEAR, STEP_ANGLE))


def main():
    rclpy.init()
    node = CalibratorNode()

    if not node.wait_for_pose():
        print('错误: 无法获取机械臂当前位姿，请确保 xArm 驱动运行中')
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f'\n当前位姿: [{", ".join(f"{v:.2f}" for v in node.current_pose)}]')
    print_help()

    step_linear = STEP_LINEAR
    step_angle = STEP_ANGLE
    corner_idx = 0
    fine_mode = False
    running = True

    try:
        while running and rclpy.ok():
            # Spin to update pose
            rclpy.spin_once(node, timeout_sec=0.02)

            key = get_key_blocking(0.05)

            if key is None:
                continue

            # Movement keys
            dx = dy = dz = droll = dpitch = dyaw = 0.0

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
                if fine_mode:
                    dyaw = step_angle
                else:
                    # Record current corner
                    node.record_current(corner_idx)
                    corner_idx += 1
                    if corner_idx >= 4:
                        print('\n  4 个角点已全部记录! 按 P 计算变换矩阵')
                    continue
            elif key in ('f', 'F'):
                dyaw = -step_angle
            elif key in ('t', 'T'):
                droll = step_angle
            elif key in ('g', 'G'):
                droll = -step_angle
            elif key == 'z':
                fine_mode = not fine_mode
                if fine_mode:
                    step_linear = max(0.5, step_linear / 2)
                    step_angle = max(0.005, step_angle / 2)
                    print(f'\n  🔍 Z轴微调模式: 步长 {step_linear:.1f}mm / {step_angle:.3f}rad')
                else:
                    step_linear = STEP_LINEAR
                    step_angle = STEP_ANGLE
                    print(f'\n  🔄 普通模式: 步长 {step_linear:.1f}mm / {step_angle:.3f}rad')
                continue
            elif key == ' ':
                node.record_current(corner_idx)
                corner_idx += 1
                if corner_idx >= 4:
                    print('\n  4 个角点已全部记录! 按 P 计算变换矩阵')
                continue
            elif key in ('+', '='):
                step_linear = min(50.0, step_linear * 2)
                step_angle = min(0.2, step_angle * 2)
                print(f'\n  🔼 步长增大: {step_linear:.1f}mm / {step_angle:.3f}rad')
                continue
            elif key in ('-', '_'):
                step_linear = max(0.1, step_linear / 2)
                step_angle = max(0.001, step_angle / 2)
                print(f'\n  🔽 步长减小: {step_linear:.1f}mm / {step_angle:.3f}rad')
                continue
            elif key in ('p', 'P'):
                M = node.compute_transform()
                if M is not None:
                    node.print_transform_code(M)
                continue
            elif key in ('h', 'H'):
                print_help()
                continue
            elif key in ('\x03', '\x1b'):  # Ctrl+C or Esc
                running = False
                break
            else:
                continue

            # Execute movement if any
            if any([dx, dy, dz, droll, dpitch, dyaw]):
                node.move_relative(dx, dy, dz, droll, dpitch, dyaw)
                # Print current command
                dir_str = []
                if dx: dir_str.append(f'X{"+" if dx>0 else "-"}{abs(dx):.1f}')
                if dy: dir_str.append(f'Y{"+" if dy>0 else "-"}{abs(dy):.1f}')
                if dz: dir_str.append(f'Z{"+" if dz>0 else "-"}{abs(dz):.1f}')
                if droll: dir_str.append(f'Roll{"+" if droll>0 else "-"}{abs(droll):.3f}')
                if dyaw: dir_str.append(f'Yaw{"+" if dyaw>0 else "-"}{abs(dyaw):.3f}')
                print(f'  Move: {", ".join(dir_str):<40s} '
                      f'Pose: ({node.current_pose[0]:.1f}, {node.current_pose[1]:.1f}, '
                      f'{node.current_pose[2]:.1f}), yaw={node.current_pose[5]:.2f}')

    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN,
                          termios.tcgetattr(sys.stdin.fileno()))
        node.destroy_node()
        rclpy.shutdown()
        print('\n退出。')


if __name__ == '__main__':
    main()
