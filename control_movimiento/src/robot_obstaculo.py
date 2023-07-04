#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from control_movimiento.msg import ArrayXY
from geometry_msgs.msg import Twist
import tf
import time

def rotFromQuat(q):
    """ q = [ex, ey, ez, ew] """
    return np.array([[2.*(q[3]**2+q[0]**2)-1., 2.*(q[0]*q[1]-q[3]*q[2]), 2.*(q[0]*q[2]+q[3]*q[1])],
                    [2.*(q[0]*q[1]+q[3]*q[2]), 2.*(q[3]**2+q[1]**2)-1., 2.*(q[1]*q[2]-q[3]*q[0])],
                    [2.*(q[0]*q[2]-q[3]*q[1]), 2.*(q[1]*q[2]+q[3]*q[0]), 2.*(q[3]**2+q[2]**2)-1.]])

def Tlidar_wrt_base():
    """ Sistema del lidar con respecto a base_link (la base del robot) """
    tflistener = tf.TransformListener()
    T = np.eye(4)
    rospy.sleep(0.5)

    try:
        (trans, rot) = tflistener.lookupTransform('base_link', 'base_scan', rospy.Time(0))
        T[0:3,0:3] = rotFromQuat(rot)
        T[0:3,3] = trans
    except:
        pass
    # Retorna la transformación homogénea del sistema del LiDAR con respecto al
    # sistema de base del robot.
    return T


class Lidar(object):
  
  def __init__(self):
    # Crear el suscriptor al tópico del LiDAR
    topic = '/scan'
    data_class = LaserScan
    self.sub = rospy.Subscriber(topic, data_class, self.callback)

    # Crear el publicador al tópico de coordenadas cartesianas
    topic = '/lidar_xy'
    data_class = ArrayXY
    self.pub = rospy.Publisher(topic, data_class, queue_size=1)

    # Esperar 1 segundo
    rospy.sleep(0.5)
    
    # Precalcular un vector de numpy que contenga los ángulos para cada rango
    step = self.measurement.angle_increment
    start = self.measurement.angle_min
    stop = self.measurement.angle_max + step
    
    self.angles = np.arange(start, stop, step)

    # Almacenar los rangos máximo y mínimo que puede leer el LiDAR
    self.min_range = self.measurement.range_min
    self.max_range = self.measurement.range_max
   
  def callback(self, msg):
    # Callback para el suscriptor
    self.measurement = msg
        
  def get_xy(self):
    """ Retorna los valores x,y de la medición, en el sistema del LiDAR""" 
    # Obtener los rangos medidos
    registered_ranges = np.array(self.measurement.ranges)

    # Filtrar los rangos que no son válidos
    filtro = []
    for rango in registered_ranges:
        if(rango > self.min_range and rango < self.max_range):
            filtro.append(True)
        else:
            filtro.append(False)

    self.valid_ranges = registered_ranges[filtro]
    valid_angles = self.angles[filtro]

    # Convertir los rangos válidos en x, y
    x = self.valid_ranges * np.cos(valid_angles)
    y = self.valid_ranges * np.sin(valid_angles)

    return x,y

class Move_Turtlebot():

    def __init__(self):
        topic = '/cmd_vel'
        data_type = Twist
        
        # Declarar el publicador de control del robot
        self.command = rospy.Publisher(topic, data_type, queue_size=1)

        # Establecer el atributo de tipo de trayectoria
        self.trajectory = None

        # Esperar a la correcta inicialización del objeto
        rospy.sleep(0.5)

        self.Hertz = 0.2
        self.rate = rospy.Rate(self.Hertz)
        # Generación del mensaje a publicar
        self.robot_order = Twist()

    def follow_trajectory(self):

        for distance in lidar.valid_ranges:
            if(abs(distance) > 0.35):
                self.robot_order.linear.x = self.Hertz
                self.robot_order.angular.z = 0
                # Publicación del mensaje para el robo
                self.command.publish(self.robot_order)

                break

            else:
                # Regresar
                self.robot_order.linear.x = -self.Hertz*0.5
                self.robot_order.angular.z = 0.
                self.command.publish(self.robot_order)
                rospy.sleep(1.5)

                self.robot_order.linear.x = 0.
                self.command.publish(self.robot_order)
                self.rate.sleep()

                # Dar vuelta
                self.robot_order.angular.z = np.deg2rad(-self.Hertz*45)
                self.command.publish(self.robot_order)
                self.rate.sleep()

                print("Giro terminado")

                break


if __name__ == "__main__":

    # Inicializar el nodo
    rospy.init_node('nodo_robot_obstaculo')

    # Transformación del sistema del LiDAR al sistema del Robot
    T_BL = Tlidar_wrt_base()

    # Objeto que lee el escaneo
    lidar = Lidar()

    # Objeto que comanda el movimiento del robot
    movimiento = Move_Turtlebot()

    # Frecuencia a la que opera el movimiento
    Hz = 3
    rate = rospy.Rate(Hz)

    while not rospy.is_shutdown():
        
        # Coordenadas en el sistema del LiDAR
        x,y = lidar.get_xy()
        
        # Conversión a coordenadas homogéneas
        P = np.ones((4, len(x)))
        P[0,:] = x
        P[1,:] = y
        P[2,:] = np.zeros((1,len(x)))   # El LiDAR mide z=0 en su propio sistema
                
        # Conversión de las coordenadas del sistema del LiDAR al sistema del Robot
        P_robot = T_BL.dot(P)
        
        xr = P_robot[0,:]     # x en sistema del robot
        yr = P_robot[1,:]     # y en el sistema del robot

        # Construcción del mensaje de mediciones
        cartesian_coordinates = ArrayXY()
        cartesian_coordinates.x = xr
        cartesian_coordinates.y = yr

        # Publicación del mensaje de mediciones
        lidar.pub.publish(cartesian_coordinates)
        
        # Realizar movimiento
        movimiento.follow_trajectory()
     
        # Esperar
        rate.sleep() 