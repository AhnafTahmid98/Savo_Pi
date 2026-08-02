# Dependency matrix

| Dependency | PC/build | Core Pi | Edge Pi | Observer | Classification |
| --- | --- | --- | --- | --- | --- |
| Ubuntu 24.04 arm64 | optional | required | required | no | target OS |
| ROS 2 Jazzy desktop/base | required | required | required | required | ROS distribution |
| `ament_cmake`, `ament_cmake_python` | required | required | required | required | ROS build packages; `ament_python` is not the key |
| `python3-pytest` / ROS ament pytest | required | test | test | test | Ubuntu/test dependency |
| `liblgpio-dev` | no | required for encoder/GPIO hardware | no | no | Ubuntu target dependency |
| `libapriltag-dev` | build if head detector enabled | required | no | no | Ubuntu package |
| `libasound2-dev` | build speech | no | required | no | Ubuntu package |
| `libpocketsphinx-dev`, `libsphinxbase-dev` | build speech | no | required | no | Ubuntu packages |
| `pocketsphinx-en-us` | optional tests | no | required wake assets | no | Ubuntu runtime data |
| RealSense SDK/ROS wrapper | optional | no | required | optional visualization | target/ROS dependency |
| Nav2, SLAM Toolbox, robot_localization | build | required | no | visualization only | ROS packages |
| `sqlite3` | operations tests | required | optional | no | Ubuntu operations dependency |

Run `rosdep check --from-paths savo_ws/src --ignore-src` after sourcing Jazzy.
`rosdep install` failures must be reported by exact key; install scripts do not
mask the final install result. Hardware libraries remain target-machine blockers
when unavailable on the development PC.
