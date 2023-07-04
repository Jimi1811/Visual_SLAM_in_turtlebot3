#!/usr/bin/env python3
#   Este nodo se suscribe a una imagen de ROS, la convierte en una matriz de
#   OpenCV y la muestra en pantalla
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sift_matching import SIFT_detector
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class Cam(object):
  def __init__(self, topic_name="camera_frame"):
    self.bridge = CvBridge()
    self.image = np.zeros((10,10))
    isub = rospy.Subscriber(topic_name, Image, self.image_callback)

  def image_callback(self, img):
    self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")

  def get_image(self):
    return self.image


class StereoProcess(object):
  def __init__(self, RCamera, LCamera, config):
    self.rCamera = RCamera
    self.lCamera = LCamera
    self.minDisparity = config['minDisparity']
    self.numDisparities = config['numDisparities']
    self.blockSize = config['blockSize']
    self.P1 = config['P1']
    self.P2 = config['P2']
    self.width = 256*2
    self.height = 256*2


  def process(self):
    self.rImage = np.uint8(self.rCamera.get_image())
    self.lImage = np.uint8(self.lCamera.get_image())
    self.stereo = cv2.StereoSGBM_create(
        minDisparity = self.minDisparity,
        numDisparities = self.numDisparities,
        blockSize = self.blockSize,
        P1 = self.P1,
        P2 = self.P2
      )

    self.disparity = self.stereo.compute(
        self.lImage,  #cv2.resize(self.lImage, (self.width, self.height)),
        self.rImage   #cv2.resize(self.rImage, (self.width, self.height))
      ).astype(np.float32)/16.0

    self.imDisp = (self.disparity - self.minDisparity)/self.numDisparities

    sobel_x = cv2.Sobel(self.imDisp, -1, 1, 0, ksize=5)
    sobel_y = cv2.Sobel(self.imDisp, -1, 0, 1, ksize=5)
    gradient = np.sqrt(sobel_x**2 + sobel_y**2)
    blur_gradient = cv2.GaussianBlur(gradient, (21,21), 0)

    # Aplicar un umbral para filtrar el ruido
    threshold = 3
    denoised_image = np.where(blur_gradient < threshold, 255, 0).astype(np.uint8)
    #min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(self.imDisp)
    #print(max_val)
    
    result = cv2.bitwise_and(self.imDisp, self.imDisp, mask= denoised_image)
    self.image_255 = cv2.multiply(result, 255)
    self.image_255 = cv2.convertScaleAbs(self.image_255)
    


    return self.imDisp, result, self.image_255
    



  def get_keypoints(self, clusters):
    pixel_values = self.image_255.reshape((-1, 1))
    pixel_values = np.float32(pixel_values)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.2)
    compactness, labels, (centers) = cv2.kmeans(pixel_values, clusters, None, criteria, 3, cv2.KMEANS_RANDOM_CENTERS)

    centers = np.uint8(centers)
    centroids = np.sort(centers.flatten())    
    
    labels = labels.flatten()
    segmented_image = centers[labels]
    segmented_image = segmented_image.reshape(self.image_255.shape)


    near_group = np.where(segmented_image == centroids[-1])
    far_group = np.where(segmented_image == centroids[-2])


    near_y = int(sum(near_group[0])/len(near_group[0]))
    near_x = int(sum(near_group[1])/len(near_group[1]))
    
    far_y = int(sum(far_group[0])/len(far_group[0]))
    far_x = int(sum(far_group[1])/len(far_group[1]))

    self.near = (near_x, near_y)
    self.far = (far_x, far_y)

    return self.near, self.far

  

def publish_pointcloud(array, mode='normal'):
    # Crea el encabezado del mensaje
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'  # Reemplaza 'base_link' con el frame ID correcto
    
    if mode == 'normal':
        # Crea una lista de puntos (x, y, z) para el PointCloud
        points = [(0.0, 0.0, 0.3), (0.0, 0.0, 0.4), (0.0, 0.0, 0.5)]  # Reemplaza con tus propios puntos
        
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



def fill_missing_values(image, kernel_shape):
    # Create a copy of the original image to store the modified version
    filled_image = image.copy()
    
    # Get the dimensions of the image
    height, width = image.shape
    
    # Calculate the padding size based on the kernel shape
    padding = kernel_shape[0] // 2
    
    # Iterate over each pixel in the image
    for i in range(padding, height - padding):
        for j in range(padding, width - padding):
            # Check if the current pixel has a missing value (i.e., pixel value is 0)
            if image[i, j] == 0:
                # Get the neighborhood pixels using the kernel shape
                neighborhood = image[i - padding : i + padding + 1, j - padding : j + padding + 1]
                
                # Calculate the mean of the neighborhood pixels
                mean_value = np.mean(neighborhood)
                
                # Set the missing value in the filled image to the calculated mean
                filled_image[i, j] = mean_value
    
    return filled_image


def publish_pointcloud(coord , color, mode='normal'):
    # Crea el encabezado del mensaje
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'camera_link'  # Reemplaza 'base_link' con el frame ID correcto
    
    if mode == 'normal':
        points = [(coord[0,j], coord[1,j], coord[2,j]) for j in range(coord.shape[1]) ]

        # Crea una lista de puntos (x, y, z) para el PointCloud
        #points = [(0.0, 0.0, 0.3), (0.0, 0.0, 0.4), (0.0, 0.0, 0.5)]  # Reemplaza con tus propios puntos
        
        # Crea el mensaje de PointCloud2
        pointcloud = pc2.create_cloud_xyz32(header, points)
    
        # Publica el mensaje en el topic
        pub.publish(pointcloud)

    
    if mode == 'rgb':
        color = color[:, 65:355]
        
        colorR = np.array(color[:,:,0], dtype=np.int32)
        colorG = np.array(color[:,:,1], dtype=np.int32)
        colorB = np.array(color[:,:,2], dtype=np.int32)

        pC = (colorR << 16) | (colorG << 8) | colorB

        pC = pC.flatten()
        #print(pC.shape)
        #print(coord.shape)
        # Crea una lista de puntos (x, y, z, r, g, b) para el PointCloud RGB
        points = [(coord[0,j], coord[1,j], coord[2,j], pC[j]) for j in range(coord.shape[1]) ]


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


def get_depth(image):
  image = image[:,70:]
  #print(image.shape)
  
  #mask = np.where(im < 10, 1, np.zeros(image.shape))
  #disparity_map = np.where(image < 10, 255, image)
  disparity_map = image

  focal_length = 500*2  # Longitud focal de la cámara
  baseline = 0.1  # Distancia entre las cámaras estéreo

  disparity_to_depth = lambda disparity: (focal_length * baseline) / (disparity + 1e-3)
  depth_map = np.array(disparity_to_depth(disparity_map), dtype=np.float32)
  #masked_distance_map = np.where(mask == 1, 0, depth_map)

  y, x = np.indices(disparity_map.shape)
  x3d = (x - disparity_map.shape[1] / 2) * (depth_map / focal_length * 4 ) 
  y3d = (y - disparity_map.shape[0] / 2) * (depth_map / focal_length * 4 ) 
  z3d = depth_map 

  return np.array([x3d.flatten(), y3d.flatten(), z3d.flatten()])


if __name__ == '__main__':

  # Inicializar el nodo de ROS
  rospy.init_node('camera_node')
  # Objeto que se suscribe al tópico de la cámara
  topic_name_right_camera = "/stereo/right/image_raw"
  topic_name_left_camera = "/stereo/left/image_raw"
  
  right_cam = Cam(topic_name_right_camera)
  left_cam = Cam(topic_name_left_camera)

  config = {
    'minDisparity' : 0,
    'numDisparities' : 64 - 0, # divisible en 16
    'blockSize' : 11, # impar entre 3-11
    'P1' : 8*3*11*11 , # 8*number_of_image_channels*blockSize*blockSize 
    'P2' : 16*3*11*11 # 32*number_of_image_channels*blockSize*blockSize 
  }

  stereo = StereoProcess(right_cam, left_cam, config)
  #SIFT = SIFT_detector(max_keypoints = 2)

  # Tópico para publicar una imagen de salida
  topic_pub = 'image_out'
  pubimg = rospy.Publisher(topic_pub, Image, queue_size=10)
  #cam = Cam(topic_pub)



  pub = rospy.Publisher('pointcloud_topic', PointCloud2, queue_size=10)
    

  # Frecuencia del bucle principal
  freq = 10
  rate = rospy.Rate(freq)
  im = np.array(0)
  # Bucle principal
  while not rospy.is_shutdown():
    
    # Obtener la imagen del tópico de ROS en formato de OpenCV
    Ir = right_cam.get_image()
    Il = left_cam.get_image()
    
    # Realizar algún tipo de procesamiento sobre la imagen
    
    imDisp, _, stereo_scaled= stereo.process()
    #fullyScaled = fill_missing_values(stereo_scaled, (35,35))
    
    #near_p, far_p = stereo.get_keypoints(3)
    
    #cv2.circle(stereo_scaled, (near_p[0],near_p[1]), 10,(0), -1)
    #cv2.circle(stereo_scaled, (far_p[0],far_p[1]), 10,(255), -1)

    #publish_pointcloud(get_depth(stereo_scaled), None, mode='normal')
    if (Ir.shape == (240, 360, 3)):
      depth = get_depth(stereo_scaled)
      #print(depth.shape)
      publish_pointcloud(depth, Ir, mode='normal')
      
      #cv2.imshow("Stereo Disparity", stereo_scaled)
      #cv2.imshow("Left", Ir)
      #cv2.imshow("Right", Il)
      #cv2.waitKey(1)

    
    rate.sleep()

cv2.destroyAllWindows()
