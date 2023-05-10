# MSc. Thesis

**(Work in Progress)**

[![ROS 2 CI](https://github.com/emilravn/msc-thesis/actions/workflows/ros.yaml/badge.svg)](https://github.com/emilravn/msc-thesis/actions/workflows/ros.yaml)
<!-- ABOUT THE PROJECT -->

## Hardware

### Bill of Materials (BOM)

The following components were used for the robot assembly:

| Item No. | Part Name                                                                                  | Quantity | Unit Price | Total Price |
| -------- | ------------------------------------------------------------------------------------------ | -------- | ---------- | ---------   |
| 1        | [Raspberry Pi 4 (4 GB)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)      | 1        | DKK529     | DKK529.00   |
| 2        | SanDisk SD Card (32 GB)                                                                    | 1        | DKK99.00   | DKK99.00    |
| 3        | SanDisk SD Card (32 GB)                                                                    | 1        | DKK99.00   | DKK99.00    |
|          | **Total**                                                                                  |          |            | **DKK128.00** |

## Development

This section will walk you through setting up the development environment for ROS 2 using Visual Studio Code. The environment is based on the [ROS2 workspace template](https://github.com/athackst/vscode_ros2_workspace) by [athackst](https://github.com/athackst) with some minor tweaks to remove C++ as the robot is fully written in Python. Additionally, some of the settings `.devcontainer` configuration file has been altered due to recent updates to Visual Studio Code.

### Prerequisites

The following are necessary to setup the development environment:

* [Docker](https://docs.docker.com/engine/install/)
* [Visual Studio Code](https://code.visualstudio.com/)
* [Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Raspberry Pi 4 Setup

The Raspberry Pi 4 is based on the [Raspberry Pi with ROS 2 and the real-time kernel](https://github.com/ros-realtime/ros-realtime-rpi4-image) image and modified during setup with the Raspberry Pi Imager software to have different a hostname and password, and to connect it to Wi-Fi and allow SSH connections. As of May 2023, the image is based off the [ROS 2 Humble distribution](https://docs.ros.org/en/humble/index.html#) with Ubuntu 22.04.01 and the real-time kernel (PREEMPT_RT) pre-installed.

The credentials during the thesis for the robot:

| Hostname     | Username    | Password             | Access Point  | Access Point (Password) |
| ------------ | ----------- | ------------------   | ----------    | ----------------------  |
| `sfr.local`  | `sfr.local` | `sFRsuperduper@.`    | `SFR`         | `sFRsuperduper@`        |

To enable SSH connection via hostname you should install **avahi-daemon** on the Raspberry Pi 4. 

### Workflow

The [robot_control.sh](https://github.com/emilravn/msc-thesis/blob/206302bcd523076b146bc1268338234476f9b547/robot_control.sh) script is used to transferring code and to communicate with the robot at ease.
  * This becomes much simpler by copying your public SSH key to the Raspberry Pi 4 with `ssh-copy-id -i </path/to/key sfr@sfr.local`. 

If on a Windows machine you should also install the **Bonjour Print Service** from Apple which enables you to discover it by hostname instead of IP-address only.

### Monitor CPU utilization, memory consumption, temperature and generate statistic reports

### Installation

Add example on how to install on node.

<!-- USAGE EXAMPLES -->

## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.
