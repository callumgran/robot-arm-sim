# Robot Arm Simulation

This repository provides a ROS2-based robot arm simulation using Docker for cross-platform compatibility. It hopefully supports Linux, Windows (WSL2+WSLg), and macOS.

## Prerequisites

Ensure you have the following installed:

### Linux
- [Docker](https://docs.docker.com/engine/install/)
- [Docker Compose](https://docs.docker.com/compose/install/)

### Windows (WSL2 + WSLg)
- Windows 10/11 with [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install)
- [WSLg](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps) (Windows Subsystem for Linux GUI) enabled
- Docker Desktop with WSL2 backend enabled

### macOS
- [Docker](https://docs.docker.com/desktop/install/mac/)
- [XQuartz](https://www.xquartz.org/) for GUI applications (Gazebo)

## Setup

1. Clone the repository:
   ```sh
   git clone git@github.com:callumgran/robot-arm-sim.git
   cd robot-arm-sim
   ```

2. Build the Docker container:
   ```sh
   make build
   ```

3. Run the container:
   ```sh
   make run
   ```

4. Inside the container, build the ROS2 workspace:
   ```sh
   cd ~/ros2_ws
   make build
   ```

5. Run the simulation:
   ```sh
   make run
   ```

## Stopping and Cleaning

- Stop the container:
  ```sh
  make stop
  ```

- Remove the container and volumes:
  ```sh
  make remove
  ```

- Clean up Docker system:
  ```sh
  make clean
  ```

## Notes

- **Windows (WSL2 with WSLg)**: Ensure that `DISPLAY`, `WAYLAND_DISPLAY`, and `PULSE_SERVER` environment variables are set for GUI applications.
- **macOS**: XQuartz must be running for GUI-based applications.
- **Linux**: Nada 😎