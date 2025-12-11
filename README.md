# Getting Started:

### Setup Colcon Workspace

Create a colcon workspace and clone the repository into the src folder:

```bash
mkdir -p humanoid_mpc_ws/src && cd humanoid_mpc_ws/src
git clone https://github.com/NotHarshPandit/wb_humanoid_mpc.git
```

Then initialize all submodules using:

```bash
cd wb-humanoid-mpc
git submodule update --init --recursive
```
### Install Dependencies
The project supports both Dockerized workspaces (recommended) or a local installation for developing and running the humanoid MPC. 

<details>
<summary>Build & run Dockerized workspace in VS Code</summary>

We provide a [Dockerfile](https://github.com/manumerous/wb_humanoid_mpc/blob/main/docker/Dockerfile) to enable running and devloping the project from a containerized environment. Check out the [devcontainer.json](https://github.com/manumerous/wb_humanoid_mpc/blob/main/.devcontainer/devcontainer.json) for the arguments that must be supplied to the `docker build` and `docker run` commands.

For working in **Visual Studio Code**, we recommend to install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension. Then, with the root of this repository as the root of your VS Code workspace, enter `Ctrl + Shift + P` and select `Dev Containers: Rebuild and Reopen in Container` at the top of the screen. VS Code will then automatically handle calling the `docker build` and `docker run` commands for you and will reopen the window at the root of the containerized workspace. Once this step is completed, you are ready to [build and run the code](https://github.com/manumerous/wb_humanoid_mpc/tree/main?tab=readme-ov-file#building-the-mpc).

</details>

<details>
<summary> Build & run Dockerized workspace with bash scripts</summary>

This repository includes two helper scripts: `image_build.bash` builds the `wb-humanoid-mpc:dev` Docker image using the arguments defined in `devcontainer.json`. `launch_wb_mpc.bash` starts the Docker container, mounts your workspace, and drops you into a bash shell ready to build and run the WB Humanoid MPC code. Example of building docker image:
```
cd /path/to/humanoid_mpc_ws/src/wb_humanoid_mpc/docker
./image_build.bash
```
and launching the docker container:
```
cd /path/to/humanoid_mpc_ws/src/wb_humanoid_mpc/docker
./launch_wb_mpc.bash
```

</details>

<details>
<summary>Install Dependencies Locally</summary>

Make sure you have **ros2** installed on your system as e.g specified for jazzy in
the [installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

Then install all dependencies using:

```bash
envsubst < dependencies.txt | xargs sudo apt install -y
```
</details>


### Building the MPC 

Building the WB MPC consumes a significant amount of RAM. We recommend saving all open work before starting the first build. The RAM usage can be adjusted by setting the PARALLEL_JOBS environment variable. Our recommendation is:

| PARALLEL_JOBS | Required System RAM |
|--------------:|--------------------:|
| 2 (default)   |  16 GiB             | 
| 4             |  32 GiB              |
| 6             |  64 GiB              | 


```bash
make build-all
```

## Running the examples
Once you run the NMPC a window with Rviz will appear for visualization. The first time you start the MPC for a certain robot model the auto differentiation code will be generated which might take up to 5-15 min depending on your system. Once done the robot appears and you can control it via an xbox gamepad or the controls in the terminal. 

On the top level folder run:

For the **Centroidal Dynamics MPC**

```
make launch-g1-dummy-sim
```

For the **Whole-Body Dynamics MPC**

```
make launch-wb-g1-dummy-sim
```

#### Interactive Robot Control
Command a desired base velocity and root link height via **Robot Base Controller GUI** and **XBox Controller Joystick**. For the joystick it is easiest to directly connect via USB. Otherwise you need to install the required bluetooth Xbox controller drivers on your linux system. The GUI application automatically scanns for Joysticks and indicates whether one is connected. 

![robot_remote_control](https://github.com/user-attachments/assets/779be1da-97a1-4d0c-8f9b-b9d2df88384f)

#### Making the robot walk
On one terminal run the following command to open the docker workspace.
```
cd humanoid_mpc_ws/src/wb_humanoid_mpc/docker/
```

Run the docker file using: 

```bash
./launch_wb_mpc.bash 
```

In the dockerized workspace run:
```
cd src/wb_humanoid_mpc/docker/
```

Then run:

```
make launch-wb-g1-dummy-sim
```

After launching, open another terminal and open the same dockerized workspace using:

```
docker exec -it wb-mpc-dev bash
```

Initialize ROS 2 using:
```
source install/setup.bash
```

To make the robot walk, use the following commands:
```
ros2 run remote_control waypoint_publisher
```

The robot will start walking. 

To modify how far the robot walks, edit the parameters in 
```
remote_control/remote_control/waypoint_publisher.py
```
This file defines the waypoint positions; adjusting them will change the robot’s walking distance.

If you want to see the plots, open a third terminal and run:

```
docker exec -it wb-mpc-dev bash
```

To plot the contact forces, run:
```
ros2 run remote_control plot_contact_forces
```


To plot the commanded waypoints, run:
```
ros2 run remote_control plot_waypoints
```

To plot the robot's base position, run:
```
ros2 run remote_control plot_base_coordinates
```

If you make changes, you can build using two commands:

```
cd humanoid_mpc_ws/src/wb_humanoid_mpc
make build-all

cd humanoid_mpc_ws
colcon build --packages-select remote_control
```

The project report and youtube video can be found here:
https://sites.google.com/umich.edu/legged-robot-control-project/home

## Acknowledgements
I sincerely thank [Manuel Yves Galliker](https://github.com/manumerous) for the open source project.