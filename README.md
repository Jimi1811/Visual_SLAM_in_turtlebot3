# Visual_SLAM_in_turtlebot3

## Requerimientos
- Ubuntu 20.04
- ROS Noetic

## Instalación

1. Crear un workspace para correr los archivos.
   
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

3. Clonar este repositorio yvolver a compilar el workspace con los nuevos paquetes.
   ```bash
   git clone https://github.com/Jimi1811/Visual_SLAM_in_turtlebot3.git
   cd ../..
   catkin_make
   ```
   
4. Dar acceso a los códigos a correr. En cada carpeta 'src' con los códigos de cada paquete:
   ```bash
   chmod a+x *
   ``` 
