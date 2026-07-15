# Third-party components

This repository contains project-specific ROS2 packages together with a source snapshot of the UFACTORY xArm ROS2 ecosystem. The separation below is important for attribution and license compliance.

## Project task layer

The following paths implement the Tetris perception, planning, calibration, and orchestration workflow described in this repository:

- `src/block_sorter/`
- `src/color_detector/`
- `scripts/`
- `config/`
- `docs/`

These paths follow the repository-level [MIT License](LICENSE), except where a file carries a more specific notice.

## UFACTORY xArm ecosystem

| Path | Upstream project | License source |
| --- | --- | --- |
| `src/xarm_api/` | [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) | `src/LICENSE` and notices in the component |
| `src/xarm_description/` | [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) | `src/LICENSE` and notices in the component |
| `src/xarm_msgs/` | [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) | `src/LICENSE` and notices in the component |
| `src/xarm_sdk/` | [xArm-Developer/xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK) and xArm ROS2 integration | `src/xarm_sdk/cxx/LICENSE` |
| `src/uf_ros_lib/` | UFACTORY ROS support library distributed with xArm ROS2 | `src/LICENSE` and component notices |
| `src/thirdparty/` | Plugins distributed with the xArm ROS2 source tree | Notices and licenses inside the corresponding component |

The repository history starts from a bulk source import and does not preserve the exact upstream branch, tag, or commit for this snapshot. The files are therefore attributed to their upstream projects without claiming an exact revision.

## Redistribution

The root MIT license does not replace third-party licenses. When redistributing the complete repository, retain `src/LICENSE`, `src/xarm_sdk/cxx/LICENSE`, copyright notices, and any component-specific license files.
