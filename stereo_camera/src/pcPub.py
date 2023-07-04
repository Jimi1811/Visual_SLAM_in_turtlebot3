#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2

def publish_pointcloud(coord , mode='normal'):
    # Crea el encabezado del mensaje
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'  # Reemplaza 'base_link' con el frame ID correcto
    
    if mode == 'normal':
        points = [(coord[0,j], coord[1,j], coord[2,j]) for j in range(coord.shape[1]) ]

        # Crea una lista de puntos (x, y, z) para el PointCloud
        #points = [(0.0, 0.0, 0.3), (0.0, 0.0, 0.4), (0.0, 0.0, 0.5)]  # Reemplaza con tus propios puntos
        
        # Crea el mensaje de PointCloud2
        pointcloud = pc2.create_cloud_xyz32(header, points)
    
        # Publica el mensaje en el topic
        pub.publish(pointcloud)

    
    if mode == 'rgb':

        red = 255
        green = 255
        blue = 0

        # Empaqueta los componentes RGB en un valor de 24 bits
        packed_color = (red << 16) | (green << 8) | blue

        # Crea una lista de puntos (x, y, z, r, g, b) para el PointCloud RGB
        points = [(0.0, 0.3, 0.0, packed_color), 
                  (0.0, 0.4, 0.0, packed_color), 
                  (0.0, 0.5, 0.0, packed_color)]  # Reemplaza con tus propios puntos
    
        #points = [(0.0, 0.3, 0.0, 255, 0, 0), (0.0, 0.4, 0.0, 0, 255, 0), (0.0, 0.5, 0.0, 0, 0, 255)]  # Reemplaza con tus propios puntos


        # Define las características del campo del PointCloud RGB
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField('rgb', 16, PointField.FLOAT32, 1),
            ]
        
        # Crea el mensaje de PointCloud2 con RGB
        pointcloud = pc2.create_cloud(header, fields, points)
        # Publica el mensaje en el topic
        pub.publish(pointcloud)

if __name__ == '__main__':

    rospy.init_node('pointcloud_publisher')
    pub = rospy.Publisher('pointcloud_topic', PointCloud2, queue_size=10)
    rate = rospy.Rate(10)
    im = cv2.imread('xd.png', cv2.IMREAD_GRAYSCALE)
    mask = np.where(im < 50, 1, np.zeros(im.shape))
    disparity_map = np.where(im < 1, 255, im)
    focal_length = 500*3  # Longitud focal de la cámara
    baseline = 0.1  # Distancia entre las cámaras estéreo

    disparity_to_depth = lambda disparity: (focal_length * baseline) / (disparity)
    depth_map = np.array(disparity_to_depth(disparity_map), dtype=np.float32)
    #masked_distance_map = np.where(mask == 1, 0, depth_map)

    y, x = np.indices(disparity_map.shape)
    x3d = (x - disparity_map.shape[1] / 2) * depth_map / focal_length
    y3d = (y - disparity_map.shape[0] / 2) * depth_map / focal_length
    z3d = depth_map



    while not rospy.is_shutdown():



        #imagen = np.array([x3d.flatten(), y3d.flatten(), z3d.flatten()])

        # imagen.shape = (3,19)
        #publish_pointcloud(imagen, mode='normal')

        rate.sleep()