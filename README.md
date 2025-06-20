#ROM_Project

How to launch?

colcon build

source install/setup.bash (en cada terminal)

ros2 launch turtlebot3_coppeliasim project_model.launch.py  //launch main

ros2 launch cartografia_pkg localization_launch.py map:=map.yaml  //añadir mapa y y publicar posiciones acmlxOdometria

ros2 run teclado_modo teclado_modo_node //app para controlar modos y mover al robot

