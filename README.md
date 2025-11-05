# MotionEditor

> A lightweight C++ utility for loading, editing, and saving YAML-based robot motion files.
> It supports frame lookup, joint editing, and joint-name–to-ID mapping while preserving the original YAML structure.
---

### Build
``` bash
cd ~/colcon_ws
colcon build --packages-select motion_editor
source install/setup.bash
```
### Run Test
``` bash
ros2 run motion_editor test_node
```
### Minimal Usage
``` c
#include "motion_editor/motion_editor.hpp"

auto me = std::make_shared<MotionEditor>();
me->loadFromFile("path/to/motion.yaml");

auto names = me->listStepNames();
auto f = me->getFrame("3");

JointPosMap q;
q["rotate_0"] = 0.1;
me->editFourArmJoints("3", q);

me->saveToFile("path/to/motion.yaml");
```

### Folder Structure
``` bash
motion_editor/
├── motion/              # robot motion files for test run
├── motion_editor/       # Library source
└── test_code/           # Example usage
```
