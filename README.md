# lanelet2_route_planning

<p align="center">
  <a href="https://github.com/openads-project"><img src="https://img.shields.io/badge/OpenADS-ffff00"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/openads-project/lanelet2_route_planning/releases/latest"><img src="https://img.shields.io/github/v/release/openads-project/lanelet2_route_planning"/></a>
  <a href="https://github.com/openads-project/lanelet2_route_planning/blob/main/LICENSE"><img src="https://img.shields.io/github/license/openads-project/lanelet2_route_planning"/></a>
  <br>
  <a href="https://github.com/openads-project/lanelet2_route_planning/actions/workflows/docker-ros.yml"><img src="https://github.com/openads-project/lanelet2_route_planning/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/lanelet2_route_planning/actions/workflows/industrial_ci.yml"><img src="https://github.com/openads-project/lanelet2_route_planning/actions/workflows/industrial_ci.yml/badge.svg"/></a>
  <a href="https://openads-project.github.io/lanelet2_route_planning"><img src="https://github.com/openads-project/lanelet2_route_planning/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/lanelet2_route_planning/actions/workflows/consistency.yml"><img src="https://github.com/openads-project/lanelet2_route_planning/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

**TODO: Repository tagline/description**

TODO: High-level repository introduction paragraph

**🚀 [Quick Start](#-quick-start)** | **🧑‍💻 [Development](#-development)** | **📝 [Documentation](#-documentation)**

> [!IMPORTANT]
> This repository is part of [🚗 ***OpenADS***](https://github.com/openads-project), the *Open Automated Driving Stack*.


<!-- <img src="TODO: teaser image/gif" width=800> -->


## 🚀 Quick Start

1. Start a container of the pre-built runtime image.
    ```bash
    docker run --rm -it ghcr.io/openads-project/lanelet2_route_planning:latest bash
    ```
1. Inside the container, launch the pre-built nodes.
    ```bash
    ros2 launch lanelet2_route_planning lanelet2_route_planning_launch.py
    ```

## 🧑‍💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/openads-project/lanelet2_route_planning.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/openads-project/openads-dev-environment) submodule containing development environment configuration.
    ```bash
    cd lanelet2_route_planning
    git submodule update --init --recursive
    ```
1. Open the repository in [Visual Studio Code](https://code.visualstudio.com).
    ```bash
    code .
    ```
1. Install the recommended VS Code extensions.
    > *Ctrl+Shift+P / Extensions: Show Recommended Extensions / Install Workspace Recommended Extensions (Cloud Download Icon)*
1. Reopen the repository in a [Dev Container](https://code.visualstudio.com/docs/devcontainers/containers).
    > *Ctrl+Shift+P / Dev Containers: Rebuild and Reopen in Container*

### Build

> *Ctrl+Shift+B*

or

```bash
colcon build
```

### Run Tests

> *Ctrl+Shift+P / Tasks: Run Test Task*

or

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon test
colcon test-result --verbose
```


## 📝 Documentation

- [Source Code Documentation](https://openads-project.github.io/lanelet2_route_planning)
- Package Documentation
  - [lanelet2_route_planning](lanelet2_route_planning/README.md)
  - [plan_route_action_client](plan_route_action_client/README.md)


## 🙏 Acknowledgements

TODO: Project/funding acknowledgements
