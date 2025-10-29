sudo apt update
sudo apt install ros-humble-joint-state-publisher
sudo apt update
sudo apt install ros-humble-urdf-launch

export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/user/ros2_ws/src/armando_description/meshes

//con questa riga vengono eseguiti automaticamente anche i controllori
ros2 launch armando_gazebo armando_world.launch.py

//installate sempre prima rqt perchè probabilmente non lo tenete
sudo apt-get install ros-humble-rqt-controller-manager
//e poi eseguite rqt e vi compaiono i controllori
ros2 run rqt_controller_manager rqt_controller_manager

//QUESTO ARRIVA FINO AL PUNTO 3 
ros2 run rqt_image_view rqt_image_view

//PER MUOVERE IL ROBOT, USA QUESTO COMANDO CON LA CAMERA ACCESA
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [0.0, 1.44, 0.0, 0.0]
}"

//per selezionare il controllo
ros2 launch armando_gazebo armando_world.launch.py choice_controll:=1

