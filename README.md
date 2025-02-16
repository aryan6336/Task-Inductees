# Task
# ROS 2 Project Setup and Commands

This repository contains a ROS 2 project with steps to clone, build, and launch nodes including `turtlesim` and `turtledraw`.

---

## Cloning the Repository

```bash
git clone <repository_url>
cd <repository_name>
```

---

## Building the Workspace

```bash
colcon build
```

---

## Sourcing the Setup File

```bash
source install/setup.bash
```

> **Note:** Make sure to source this every time you open a new terminal or add it to your `.bashrc` for convenience.

---

## Launching `turtlesim` Node

```bash
ros2 run turtlesim turtlesim_node
```

---

## Launching `turtledraw` Node

```bash
ros2 run <your_package_name> turtledraw
```

Replace `<your_package_name>` with the name of your ROS 2 package that contains the `turtledraw` node.

---

### Additional Information

- **ROS 2 Distribution:** Humble Hawksbill
- **Tested On:** Ubuntu 22.04 LTS
- **Dependencies:** Ensure all dependencies are installed using `rosdep`:
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```

---

Happy Coding! 🚀

