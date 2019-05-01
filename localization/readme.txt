Para crear un nuevo mapa:

    Para eso tienes que usar un solo robot
    1. roslaunch a_multirob create_one_robot.launch
    2. roslaunch localization mapping.launch
    3. roslaunch turtlebot_rviz_launchers view_navigation.launch
    
    4. roslaunch turtlebot_teleop keyboard_teleop.launch 
    (si no funciona)
    rostopic pub -r 10 /mobile_base/commands/velocity geometry_msgs/Twist '{linear: {x: 0.1}}'
    rostopic pub -r 10 /mobile_base/commands/velocity geometry_msgs/Twist '{angular: {z: 0.1}}'

    
    Cuando el mapa esté acabado:
    5. rosrun map_server map_saver -f my_map
    
    Con esos nodos tienes los suficiente


Para localizarse en un mapa:
1. roslaunch a_multirob create_one_robot.launch
2. roslaunch localization localization.lauch
3. roslaunch turtlebot_rviz_launchers view_navigation.launch

