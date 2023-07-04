# Procedimiento para correr el stereo camera

## Requerimientos
- Ubuntu 20.04
- ROS Noetic
- stereo_image_proc
- rtabmap

## Instalación

1. Ejecutar el siguiente comando para la descarga de los archivos del paquete de image_pipeline (paquete de procesamiento de stereo cameras)
   
   Ubicarse en /catkin_ws/src o /lab_ws/src

   ```bash
   git clone https://github.com/ros-perception/image_pipeline.git
   ```

2. Ubicarse en el directorio del workspace (/catkin_ws o /lab_ws). Esto construye el paquete de stereo_image_proc

    ```bash
    catkin_make --only-pkg-with-deps stereo_image_proc

    ```

3. Clonar este repositorio y volver a compilar el workspace con los nuevos paquetes.
   ```bash
   git clone https://github.com/Jimi1811/Visual_SLAM_in_turtlebot3.git
   cd ../..
   catkin_make
   ```
   
4. Dar acceso a los códigos a correr. En cada carpeta 'src' con los códigos de cada paquete:
   ```bash
   chmod a+x *
   ``` 

5. Para ejecutar todo el procedimiento ejecutar cada línea de abajo en distintos terminales.

   ```bash
   roslaunch stereo_camera maze.launch
   
   ROS_NAMESPACE=/stereo rosrun stereo_image_proc stereo_image_proc

   roslaunch rtabmap_legacy stereo_mapping.launch stereo_namespace:="/stereo" rtabmap_args:="--delete_db_on_start" rviz:=true rtabmapviz:=false
   ``` 
6. De allí, controlar el robot para que se mueve por medio de todo el mapa.

## Importante

El turtlebot presente, usado en este paquete es modificado, por lo que es preferible correr los nodos respecto a los frames del presente paquete.

Por otro lado, colocar la carpeta _my_ground_plane_ dentro del directorio /home/.gazebo/models/