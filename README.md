## LAND VEHICLE
---
- Bu program için gereksinimler
1) **ubuntu 22.04** 
2) **ROS2 Humble**
3) **Gazebo GZ Sim**

- land_vehicle kullanmak için ok tuşlarını kullanın
- command_node debug kısımlarının çalışmasını istersek: 
```
ros2 run command  command_node --ros-args --log-level Command_node:=debug
```

- Launch dosyasını çalıştırmak için:
`ros2 launch land_vehicle start_launch.py`
