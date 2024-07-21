# Vehicle Control Software

This repository contains software written for ground vehicle control. The software is designed to be used both in the Gazebo Garden simulation environment and on a real vehicle. The vehicle attempts to avoid obstacles using mounted distance sensors (Lidar). To make the system more realistic, noise has been added to the sensors in the simulation environment. Future versions will include additions for artificial intelligence (AI) and image processing. The project is not finished yet, so keep your repository up to date.

https://github.com/user-attachments/assets/59023c52-a4bc-4c01-aa8e-c8ea53fad5f9

## Contents 
[Features](https://github.com/serkanMzlm/Vehicle-Control-Sotfware?tab=readme-ov-file#features)

[Features to be Added](https://github.com/serkanMzlm/Vehicle-Control-Sotfware?tab=readme-ov-file#features-to-be-added)

[Control](https://github.com/serkanMzlm/Vehicle-Control-Sotfware?tab=readme-ov-file#control)

[Gazebo Garden](https://github.com/serkanMzlm/Vehicle-Control-Sotfware?tab=readme-ov-file#gazebo-garden)

[RViz2](https://github.com/serkanMzlm/Vehicle-Control-Sotfware?tab=readme-ov-file#rviz2)

[RQT](https://github.com/serkanMzlm/Vehicle-Control-Sotfware?tab=readme-ov-file#rqt)

[Packages And Their Tasks](https://github.com/serkanMzlm/Vehicle-Control-Sotfware?tab=readme-ov-file#packages-and-their-tasks)

###  ON THE [WIKI](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/wiki) THE PAGE
- **[System Requirements](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/wiki#system-requirements)**
- **[Installation](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/wiki#install)**
- **[Compilation](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/wiki#Compile)**
- **[Run](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/wiki#Run)**
---

### Features:
- Obstacle avoidance using a Lidar sensor.
- Adding models to the Gazebo environment using C++.
- Accessing and using packages located in different places.
- Recording camera data and camera calibration
- URDF files for RViz
- For vehicle control, RemoteXY (esp8266 NODEMCU), joystick, and keyboard can be used.

### Features to be Added:
- **Position Keeping:** The IMU sensor mounted on it will ensure that the vehicle remains continuously aware of its position during movement. Additionally, in the event of external disturbances, such as impacts, it will attempt to return to its previous position if its direction changes.
- **Sensor filtering and fusion:** Reducing noise in the sensors and using sensor fusion (EKF) to obtain more accurate results.
- **Control Methods:** One of the control methods such as PID, LQR, or MPC will be added.
- **Configure:** A menuconfig file will be added to facilitate the selection of packages.
- **Parallel Processing:** Threads will be used for parallel operations
- **Path Planning**
- **Motion Planning**
- **Mapping**
- **OpenCV:** Object detection, filtering noise in camera data, and applying masking to camera images
- **AI**

## Control

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
    - Arduino Code [nodemcu_esp8266.ino](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/blob/main/Tools/arduino/nodemcu_esp8266/nodemcu_esp8266.ino)

![remotexy](https://github.com/user-attachments/assets/29a6654e-0424-47bd-80db-8beaa7ec1d34)

- Instead of running each code individually, the launch file is executed.
```bash
ros2 launch vehicle_control_software drive_launch.py
```
- If files are to be launched individually, the path to the params file should be provided during the launch.
```bash
ros2 run commander commander_node  --ros-args --params-file /home/${USER}/Vehicle-Control-Sotfware/src/modules/vehicle_control_software/config/params.yml
```

## Gazebo Garden

![rviz](./Documentation/images/gz_sim.png)
- Simulation settings can be modified by assigning the topic name lidar for the lidar window to display Lidar data. If it is desired to hide areas where detection is not performed, unchecking the show non-hitting rays option is recommended
- To change the model in Gazebo, it is sufficient to modify the [config](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/blob/main/src/modules/vehicle_control_software/config/params.yaml#L41) file. The [gz_bridge](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/tree/main/src/modules/gz_bridge) ROS2 package enables the dynamic loading of models into the Gazebo environment. (To add the model, it needs to be located in the [models](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/tree/main/Tools/simulation/models) directory.)

**Note:** By default, the world is set to default and the model is set to marble_husky. Obstacle avoidance is implemented accordingly. Other models need to be adjusted, and a Lidar sensor needs to be added.

## RViz2
The Lidar sensor data is visualized in conjunction with the camera at the front of the vehicle, as well as the data from the joystick and the error information from obstacle avoidance.

- The **RED** arrow represents the linear velocity.
- The **GREEN** arrow represents the angular velocity.
- The **BLUE** arrow represents the result vector.

![rviz](./Documentation/images/rviz2.png)

- The Orion model has been imported into RViz using the [URDF file](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/tree/main/src/modules/vehicle_control_software/urdf)
- The fixed frame should be set to base_footprint.

![rviz](./Documentation/images/rviz2_2.png)


## RQT
Using rqt, we can observe the relationships between packets

![rviz](./Documentation/images/rosgraph.png)

## Packages And Their Tasks
The contents of the src folder
![tree](./Documentation/images/tree_2.png)

**1. [drivers:](https://github.com/serkanMzlm/Sensor-Drivers/tree/2b5228538e5d041b34f09c7f603990f4b1cd3ab6)** Libraries necessary for the operation of sensors are included. **(Not required in the simulation environment.)**

**2. [include:](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/tree/main/src/include)** Packages containing libraries that can be used across all packages are included. These packages cannot be executed with `ros2 run` they can only be added as libraries to other packages. During system setup, the packages in this directory are built first, and after being included in the system using the .install/setup.bash command, the other packages are built.
```cmake
find_package(geometric_operations REQUIRED) 
```
**3. [src:](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/tree/main/src)**
- **camera_calibration:** It allows for the calibration of the camera by taking images recorded by the camera.
- **camera_streamer:** The camera allows for the recording and playback of the broadcasted video.
- **commander:** The package that enables the vehicle to avoid obstacles around it operates by receiving and processing Lidar data broadcasted from the vehicle. The rules necessary for obstacle avoidance to function can be configured through the [config](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/blob/main/src/modules/vehicle_control_software/config/params.yaml#L6) file.
- **control_unit:** For the vehicle to move, a movement command must be sent by the user. This package allows configuring the control options for the vehicle according to the `control_unit` option specified in the [config](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/blob/main/src/modules/vehicle_control_software/config/params.yaml) file.
- **gz_bridge:** Intervention in the Gazebo simulation from an external C++ file is enabled. This package allows the specified model to be added after the Gazebo simulation environment is launched. Allows the position of the model or the model itself to be changed directly during runtime.
- **save_image:** Allows the data received from the camera to be saved as images.
- **vehicle_control_software:** It is created to manage the directories that should be in the build folder. Additionally, it contains config and launch files

**4. [arduino](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/tree/main/Tools/arduino):** Contains the code for the ESP8266 NodeMCU with RemoteXY installed.
**5. [simulation](https://github.com/serkanMzlm/Vehicle-Control-Sotfware/tree/main/Tools/simulation):** It is the package that contains the models and worlds in the simulation environment.

