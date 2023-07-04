# Visual_SLAM_in_turtlebot3

## Requerimientos
- Ubuntu 20.04
- ROS Noetic
- stereo_image_proc
- rtabmap

## Instalación

1. Crear un workspace para correr los archivos. No se olvide de agregar el workspace en el .bashrc.
   
   ```bash
   cd
   mkdir proyecto_ws
   cd proyecto_ws
   mkdir src
   catkin_make

   ```

2. Crear una carpeta con nombre 'Turtlebot3' para descargar todos los paquetes necesarios para correr el robot.

    ```bash
    cd src
    mkdir turtlebot3
    cd turtlebot3
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

    ```

3. Clonar este repositorio y volver a compilar el workspace con los nuevos paquetes.
   ```bash
   cd ..
   git clone https://github.com/Jimi1811/Visual_SLAM_in_turtlebot3.git
   git clone https://github.com/ros-perception/image_pipeline.git
   cd ..
   catkin_make

   ```
   
4. Dar acceso a los códigos a correr. En cada carpeta 'src' con los códigos de cada paquete:
   ```bash
   roscd control_movimiento;cd src
   chmod a+x *
   roscd odom_imu;cd src
   chmod a+x *
   roscd stereo_camera;cd src
   chmod a+x *

   ``` 

5. Para ejecutar todo el procedimiento ejecutar cada línea de abajo en distintos terminales.

   ```bash
   roslaunch stereo_camera maze.launch

   ``` 

   ```bash
   ROS_NAMESPACE=/stereo rosrun stereo_image_proc stereo_image_proc

   ``` 

   ```bash
   roslaunch rtabmap_legacy stereo_mapping.launch stereo_namespace:="/stereo" 

   ``` 
   
   ```bash
   rtabmap_args:="--delete_db_on_start" rviz:=true rtabmapviz:=false

   ``` 

6. De allí, controlar el robot para que se mueve por medio de todo el mapa. Se puede realizar con el teleop del turtlebot.
   ```bash
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

   ```

## Importante

El turtlebot presente, usado en este paquete es modificado, por lo que es preferible correr los nodos respecto a los frames del presente paquete.

Por otro lado, colocar la carpeta _my_ground_plane_ dentro del directorio /home/.gazebo/models/


## Calibración
Para realizar la calibración de las cámaras stereo, se usa https://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
- Deben crear el paquete de camera_calibration
   ```bash
   catkin_make --only-pkg-with-deps camera_calibration
   sudo rosdep install camera_calibration
   rosmake camera_calibration

   ```

- Ahora solo ejecutar el nodo de lectura de las imagenes de las cámaras y el siguiente comando para ejecutar el nodo de camera_calibration

   ```bash
   rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.108 right:=/my_stereo/right/image_raw left:=/my_stereo/left/image_raw right_camera:=/my_stereo/right left_camera:=/my_stereo/left
   
   ```
- Los tópicos de las cámaras dependen de cómo estas son accedidas con los tópicos. Más info en https://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
