# VR-Final-Project
This is the final project of Virtual Reality



## Requirements

- Unreal Engine 5.2
- Visual Studio 2022 (with Windows 10 SDK 10.0.19041 and latest .NET Framework SDK)
- Cesium plugin for Unreal
- Python 3.8
- WSL (with Ubuntu 20.04)
- ROS Noetic

### How to install

1. Go to Epic Games GitHub page: https://www.unrealengine.com/en-US/ue-on-github

2. Follow instructions to link your Epic account to GitHub.

3. Once linked, you’ll have access to the private UnrealEngine repo on GitHub.
    if you have linked your accounts successfully you should see this link without 404 ERROR!
    https://github.com/EpicGames

    If not linked yet go directly to:
    https://github.com/orgs/EpicGames/invitation
    and accept the Invite (if pending).

4. Log in with the GitHub account linked to Epic.

5. In the repo, switch to the branch;at the moment of publishing this code we only have Cesium plugin for version 5.0 → 5.3:
    ```bash
    5.3
    ```

6. Clone it:
    ```bash
    git clone -b 5.3 https://github.com/EpicGames/UnrealEngine.git
    cd UnrealEngine
    Setup.bat
    GenerateProjectFiles.bat
    ```

7. run:
    ```bash
    Setup.bat
    GenerateProjectFiles.bat
    ```
    if you see any errors make sure you already installed Visual Studio 2022 and both "Game development with C++" and "Desktop development with C++" are ticked.

8. open your powershell and install WSL (with Ubuntu 20.04):
    ```bash
    wsl --install -d Ubuntu-20.04
    ```
    This will:
    - Enable the Windows Subsystem for Linux
    - Download and install Ubuntu 20.04 LTS
    - Ask you to create a username + password when first launched

9. You need to add the ROS package source:
    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```
    then run these one by one:
    ```bash
    sudo apt install curl -y; sudo apt update
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    ```

10. Now you can install ROS Noetic:
    ```bash
    sudo apt install ros-noetic-desktop-full -y
    ```

    Finally, initialize rosdep (needed for dependencies):
    ```bash
    sudo apt install python3-rosdep -y
    sudo rosdep init
    rosdep update
    ```

#### Python requirements

```bash
pip install airsim msgpack-rpc-python # Airsim related
pip install geopandas shapely fiona requests numpy # Other packages
```

## Setup

#### Install Colosseum (on Windows)

To install and build Colosseum on Windows open the `Developer Command Prompt for VS 2022`:

```bash
git clone https://github.com/CodexLabsLLC/Colosseum.git
cd Colosseum
build.cmd
```

For more details follow the steps at this [link](https://microsoft.github.io/AirSim/build_windows/).

#### Setup the Unreal environment

Form the Unreal editor create a `new C++ class`, leave everything as default, then close Unreal and Visual Studio.
Copy `Unreal\Plugins` into your project's folder and add the following lines in your `<project_name>.uproject`:

```json
"Modules": [
    {
        "AdditionalDependencies": [
            "AirSim"
        ]
    }
]

"Plugins": [
    {
        "Name": "AirSim",
        "Enabled": true
    }
]
```

Also add this line to the `Config\DefaultGame.ini` file in your project's directory:

```bash
+MapsToCook=(FilePath="/AirSim/AirSimAssets")
```

Finally, right click the `.uproject` and select `Generate Visual Studio Project Files`.

Full tutorial at this [link](https://microsoft.github.io/AirSim/unreal_custenv/).

#### Launch the project

Open the `<prject_name>.sln`. In Visual Studio select "DebugGame editor" and "Win64" as build configuration.

Now you can run the project by pressing `F5`.

#### Set up Colosseum in WSL

We assume you have already installed *ROS Noetic*.
To install and build Colosseum, in your ROS workspace:

```bash
git clone https://github.com/CodexLabsLLC/Colosseum.git src/
cd src/Colosseum
./setup.sh
./build.sh
```

***Note***: you may need to delete the `ros2` folder in the Colosseum repository to avoid building problems.

Afterwards, build your workspace using `catkin_make`.

Full guide at this [link](https://microsoft.github.io/AirSim/airsim_ros_pkgs/).

#### Enable Airsim settings

To enable the Airsim settings copy the `settings.json` file in this repository in the folder `Documents/AirSim`.

```bash
cp settings.json <path_to_documents>/Documents/AirSim/settings.json
```

## Simulations

#### Launch the simulation

1. Open UE and start the game mode
2. Open a WSL terminal and launch: `roslaunch assignment_pkg assignment.launch`

***Note***: you may need to set up you ip and port. You can do it by setting them directly in the launch file or by launching the simulation with `host` and `port` as arguments. `roslaunch assignment_pkg assignment.launch`

#### Other configurations

In the launch file you can customize some parameters that will slightly change the simulation environment:
- `update_lidar_every_n_sec`: to change the update frequency of the lidar
- `weather`: to enable weather effects (see [allowed effects](https://microsoft.github.io/AirSim/apis/#weather-apis))
- `weather_value`: to set the turbulence level (from 0 to 1)

## Software Architecture
![ROS Architecture](UML_VR.drawio.png)

you can commit your updates to git using:
    ```bash
    git add README.md
    git commit -m "Update README with new information"
    git push
    ```
    you can modify commands based on what you have eddited.