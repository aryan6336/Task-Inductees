# Task
# ROS 2 Project Setup and Commands

## Execute these commands to start

```bash
git clone https://github.com/NambiarAnand/Task-Inductees.git
cd Task-Inductees/task_ws
```

---

## After editting your code

```bash
colcon build
source install/setup.bash
ros2 run turtlesim turtlesim_node
```
> **Note:** Make sure to source this every time you open a new terminal or add it to your `.bashrc` for convenience.
---

## On new Terminal

```bash
source install/setup.bash
ros2 run turtlesim_draw turtlesim_draw
```

> **Note:** Make sure to source this every time you open a new terminal or add it to your `.bashrc` for convenience.

---

### Additional Information

- **ROS 2 Distribution:** Humble Hawksbill
- **Tested On:** Ubuntu 22.04 LTS

---

Happy Coding! 🚀

