#!/bin/bash

# Defina o caminho para o arquivo do mapa
MAP_FILE_PATH='./src/nav/pkg/launch/my-map.yaml'

# Comando para iniciar o ambiente do Gazebo com o TurtleBot3
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &

# Comando para iniciar a navegação com o TurtleBot3 usando o arquivo do mapa
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=${MAP_FILE_PATH} &

# Substitua <seu_pacote> e <executável> pelos valores reais
ros2 launch nav_pkg nav_launch.py