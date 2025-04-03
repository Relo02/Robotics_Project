# Docker-based ROS1 Workspace for Robotics Course

## Overview
This project provides a Dockerized environment for developing and running ROS1-based applications for the robotics course. It includes a pre-built Docker image that simplifies the setup process and ensures consistency across different machines.

## Folder Structure
- `docker/` - Contains the Docker-related files.
  - `Dockerfile` - Defines the Docker image. Only needed for custom builds.
  - `build.sh` - Script to build the Docker image locally.
- `start.sh` - Script to start the Docker container. It pulls the pre-built image from the repository and mounts relevant folders.
- `catkin_ws/` - Workspace for ROS development.
- `data/` - Storage folder for data.
- `macos_users/` - Start scripts for MacOs userser, using noVNC as a server to stream a virtual desktop.
- `windows_users/` - Start scripts for Windows userser, using noVNC as a server to stream a virtual desktop.

## Usage (Linux based systems/macos)

These instructions are for linux-based system or MacOS systems with an x-server equivalent installed, like [XQuartz](https://www.xquartz.org/) 

### Starting the Container
To start the Docker container, run:
```bash
./start.sh
```
This script pulls the pre-built image from the repository and mounts the `catkin_ws` and `data` folders to ensure persistence.

If you want to use a custom-built Docker image, modify `start.sh` accordingly.

### Connecting to the running Container

```bash
./connect.sh
```

This script can be run after `start.sh` to connect to the running container from a new terminal.


### Building a Custom Docker Image
If you need to customize the Docker image, navigate to the `docker/` folder and run:
```bash
cd docker
./build.sh
```
Then, update `start.sh` to use the newly built image instead of pulling from the repository.

## Usage (Windows based systems, using noVNC)
Navigate to the scripts folder `windows_users/`, using Windows Powershell (scripts are written for Windows Powershell, not Windows cmd)

ONLY the first time run this command to inizialize the comunication between different ROS containers

```bash
.\docker network create ros
```

First start the noVNC server:
```bash
.\windows_gui_initialize.ps1
```
Then start the ROS docker
```bash
.\start.ps1
```
To visualize GUI-based applications navigate to: [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html) 
After pressing connect you should be able to see a desktop where gui-based applications run inside docker are shown

## Usage (MacOS based systems, using noVNC)
Navigate to the scripts folder `macos_users/`

ONLY the first time run this command to inizialize the comunication between different ROS containers

```bash
.\docker network create ros
```

First start the noVNC server:
```bash
.\macos_gui_initialize.ps1
```
Then start the ROS docker
```bash
.\start.ps1
```
To visualize GUI-based applications navigate to: [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html) 
After pressing connect you should be able to see a desktop where gui-based applications run inside docker are shown

## Workspace Setup
Once inside the Docker container, you will have access to:
- A `catkin_ws` folder in your home directory to develop ROS packages.
- A `data` folder for persistent data storage.

These folders are mounted, meaning any changes made persist across container restarts.

## Notes
- Ensure Docker is installed on your system before running the scripts.
- If modifying `start.sh`, ensure it points to the correct Docker image.
- The default workflow uses the pre-built Docker image for convenience, but custom images can be built if needed.


