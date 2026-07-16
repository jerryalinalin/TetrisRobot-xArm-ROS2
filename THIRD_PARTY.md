# 第三方组件与许可边界

本仓库同时包含项目自有 ROS2 packages 与 UFACTORY xArm ROS2 生态的源码快照。明确区分两类内容，有助于正确说明贡献范围并遵守相应许可证。

## 项目任务层

以下路径实现了仓库所描述的俄罗斯方块感知、规划、标定和流程编排：

- `src/block_sorter/`
- `src/color_detector/`
- `scripts/`
- `config/`
- `docs/`

除非文件中存在更具体的许可声明，上述路径遵循仓库根目录的 [MIT License](LICENSE)。

## UFACTORY xArm 生态

| 路径 | 上游项目 | 许可证来源 |
| --- | --- | --- |
| `src/xarm_api/` | [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) | `src/LICENSE` 及组件内声明 |
| `src/xarm_description/` | [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) | `src/LICENSE` 及组件内声明 |
| `src/xarm_msgs/` | [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) | `src/LICENSE` 及组件内声明 |
| `src/xarm_sdk/` | [xArm-Developer/xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK) 与 xArm ROS2 integration | `src/xarm_sdk/cxx/LICENSE` |
| `src/uf_ros_lib/` | 随 xArm ROS2 分发的 UFACTORY ROS support library | `src/LICENSE` 及组件内声明 |
| `src/thirdparty/` | 随 xArm ROS2 源码树分发的 plugins | 对应组件目录中的声明与许可证 |

仓库历史从一次批量源码导入开始，没有保留该快照对应的准确上游 branch、tag 或 commit。因此本文按上游项目标注文件来源，不对具体版本作无法验证的声明。

## 再分发说明

仓库根目录的 MIT License 不会替代第三方许可证。完整分发本仓库时，应保留 `src/LICENSE`、`src/xarm_sdk/cxx/LICENSE`、著作权声明以及各组件专用的许可证文件。
