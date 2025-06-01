# Martian Mines ROS2

Workflow:
1) Create new branch fork from develop branch with problem name,
2) solve problem,
3) create pull request to develop. 

### Requirements
- **Docker** (Docker CLI recommended)
  - Add your user to the Docker group to grant the necessary permissions; otherwise, you must prefix commands with sudo.
  
- **Visual Studio Code** (recommended) with the following extensions:
  - [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
  - [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Docker build
```bash
docker build -t highflyers/martian_mines_ros2:latest . --build-arg USER_UID=$(id -u)
```

### Run for development
Running docker with GPU support requires [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), but if you don't have NVIDIA GPU, you can remove the `--gpus=all` flag in `devcontainer.json` and run this container only with CPU.

Open repository in vscode:
```bash
cd /path/to/martian_mines_ros2/
code .
```
Click `Ctrl+Shift+P` and select `Dev Containers: Rebuild and Reopen in Container`. 
This will open the repository in the container and you can start developing.

To rebuild workspace use shortcut `Ctrl+Shift+B` in the vscode.


## Running the Mission in Simulation
To run the mission in simulation, ensure that both the simulation and mission containers share the same ROS_DOMAIN_ID.

Start the Simulation Container:
Launch the simulation container using the appropriate command described in the uav_sim_ros2 repository.

Wait for Gazebo and QGroundControl:
Ensure both Gazebo and QGC are running properly.

Launch the Mission:
In a separate terminal, run the mission launch command:
```bash
ros2 launch martian_mines mission.launch.py
```

## Running the Mission on Real Hardware (Jetson)
To run the mission on the Jetson, you’ll need three terminal sessions using the same Docker image (one for the mission, one for MicroXRCE-DDS, and one to start the mission).

Step 1: Start the Docker Container
Use the following command to start the container:
```bash
docker run --network=host --ipc=host \
  --device=/dev/ttyACM0 --group-add dialout \
  --volume /dev/bus/usb:/dev/bus/usb --privileged \
  -it martian-mines-ros2-system /bin/bash
```
### Note:
The flags --device=/dev/ttyACM0 and --group-add dialout are only needed for the container running the MicroXRCE-DDS agent.
The serial device (/dev/ttyACM0) may change on reboot, so verify the correct device each time. If you switch from a USB-UART converter to Jetson's native UART pins, the device path may also change.

Ensure you're using the martian-mines-ros2-system container, which is the latest version built on the Jetson. If unsure, rebuild the system with your preferred container name.

Also verify ROS_DOMAIN_ID in each container if running in seperate once.

### Step 2: Run the MicroXRCE-DDS Agent
In the first terminal, run:
```bash
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 115200
```
Adjust the --dev flag if your serial device path differs.

### Step 3: Build and Launch the System
In the second terminal:

Navigate to the martian_mines_ros2 workspace:
```bash
cd ~/ws/src/martian_mines_ros2/
```

Build the system packages:
```bash
source ./scripts/build.sh ../..
```

Source the workspace:
```bash
source install/setup.bash
```

Launch the mission in real-world mode:
```bash
ros2 launch martian_mines mission.launch.py real_world:=True
```

### Step 4: Start the Mission
In the third terminal, run the following command to start the mission:

```bash
ros2 topic pub --once /mission_start std_msgs/Empty "{}"
```
Once everything is running, you should observe the drone arming and mission-related status messages printed in the terminal.

### TO DO  List
- Launches
  - [x] core.launch 
  - [ ] example_precison_landing.launch 
  - [x] example_trajectory_tracker.launch 
  - [x] figure_finder.launch.py
  - [x] figures_vis.launch
  - [x] mission.launch 
  - [ ] realsense.launch 
  - [ ] rosbag_play.launch 
  - [ ] rosbag_record.launch 
  - [ ] rviz.launch 

### Throubleshoting
If you will encounter a problem with "xcb" (GUI applications won't open) just run `xhost +local:docker` on your host machine.
