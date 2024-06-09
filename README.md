# Secure Drive Vehicle

### System Requirements
- OS: [Ubuntu 22.04 Jammy Jellyfish](https://releases.ubuntu.com/jammy/)
- Simulation Program: [Gazebo Garden](https://gazebosim.org/docs/garden/getstarted)
- ROS2: [Humble Hawksbill](https://docs.ros.org/en/humble/index.html)

### Install
- Set the directory where the project will be downloaded (for example, the home directory is used in this example).

```bash
cd ~
git clone git@github.com:serkanMzlm/Secure-Drive-Vehicle.git

# A branch can be downloaded directly.
git clone git@github.com:serkanMzlm/Secure-Drive-Vehicle.git -b [branch_name]
```

### Compile
- The path to the model should be added to the GZ_SIM_RESOURCE_PAT environment variable
```bash
export GZ_SIM_RESOURCE_PATH=/home/${USER}/Secure-Drive-Vehicle/Tools/simulation/models"
```

- Optionally, to have this command automatically executed every time a terminal is opened, it can be added to the `.bashrc` file.
- If you directly added it to the `.bashrc` file, run the `.bashrc` file once

```bash
echo "export GZ_SIM_RESOURCE_PATH=/home/${USER}/Secure-Drive-Vehicle/Tools/simulation/models" >> ~/.bashrc 

source ~/.bashrc # or bash or simply restart the terminal
```
- When using the `bash` command to reload the terminal, it restarts the terminal session, while `source ~/.bashrc` reloads the .bashrc file, applying the changes for the current session

- Build the project.
    - First, build the packages located in src/include. For example, build and include the geometry_utils, num_tools, and time_utils packages in the system. We use these modules as libraries in the packages.
    
```bash
cd ~/Secure-Drive-Vehicle

colcon build --packages-select num_tools time_utils geometry_utils
. install/setup.bash
```
```bash
#Builds all files in the project.
colcon build  
```

### Run
```bash
source /opt/ros/humble/setup.bash
source ~/Secure-Drive-Vehicle/install/setup.bash
```

- To avoid entering the `source ~/Secure-Drive-Vehicle/install/setup.bash` command every time a new terminal is opened, these commands are written to the `~/.bashrc` file.
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /home/${USER}/Secure-Drive-Vehicle/install/setup.bash >> ~/.bashrc
```

- The control method is specified in the 'control_unit' variable in the params file to control the vehicle. By default, it is set to keyboard control. If desired, it can also be controlled using a joystick or an ESP8266.

1. **Keyboard controls are as follows**
```
    w
a   s   d   
    x
```
2. **Joystick control:** Control is achieved using the left joystick.
3. **ESP8266 control:** Joy data is sent to the computer's serial port via an interface prepared using [RemoteXY](https://remotexy.com/en/). 
    - To connect to the ESP8266 board, it is necessary to write the port to which the board is connected in the file_name variable in the params file. By default, it is /dev/ttyUSB0
    - The RemoteXY application is downloaded to the phone, and to connect to the access point broadcasted by ESP8266, you connect to `joy` in the Wi-Fi section. The password is `135798642`.
    - Arduino Code `Tools/arduino/nodemcu_esp8266/nodemcu_esp8266.ino`

- Instead of running each code individually, the launch file is executed.
```bash
ros2 launch secure_drive_vehicle drive_launch.py
```
- If files are to be launched individually, the path to the params file should be provided during the launch.
```bash
ros2 run commander commander_node  --ros-args --params-file /home/${USER}/Secure-Drive-Vehicle/src/modules/secure_drive_vehicle/config/params.yml
```

## NOTE
- The project is not finished yet, so keep your repository up to date.
