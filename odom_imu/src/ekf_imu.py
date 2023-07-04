#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
import numpy as np
import tf
from geometry_msgs.msg import Twist, Pose, Point, Vector3
from nav_msgs.msg import Odometry


class SuscriptorImu(object):
    def __init__(self):
        topic = 'imu'
        self.pub = rospy.Subscriber(topic, Imu, self.callback_point)
        self.imu = Imu()
        
    def callback_point(self, msg):
        self.imu = msg

    def get_imu(self):
        
        return self.imu
    
class SuscriptorCmd(object):
    def __init__(self):
        topic = '/cmd_vel'
        self.pub = rospy.Subscriber(topic, Twist, self.callback_point)
        self.vel = Twist()
        
    def callback_point(self, msg):
        self.vel = msg

    def get_vel(self):
        
        return self.vel
    
    
def prediccion_ekf(x, u,dt):
    """
    Predicción del estado usando el modelo del movimiento
    
    x - Estado anterior
    u - señal de control
    
    """
    px = x[0,0]; vx = x[1,0]; ax = x[2,0]
    py = x[3,0]; vy = x[4,0]; ay = x[5,0];theta=x[6,0]; w=x[7,0]
    vx_1=x[1,0]; vy_1=x[4,0]
    
    u_vx= u[0,0]; u_vy=u[1,0]; w_u = u[2,0]
   
    v=np.sqrt(u_vx*u_vx+u_vy*u_vy)
    
    if theta==0: theta = 0.00000000001
    # Modelo del sistema basado en velocidad
    
    
    #posiciones
    px = (px + v/w_u*(np.sin(theta+w_u*dt)-np.sin(theta)))
    py = py + v/w_u*(-np.cos(theta+w_u*dt)+np.cos(theta))
    
    #velocidades
    vx = vx + ax*dt
    vy = vy + ay*dt

    #ax = ax #(vx - vx_1)/dt
    #ay = ay #(vy - vy_1)/dt
    
    theta = theta +w_u*dt
    
    Xout = np.array([[px], [vx], [ax], [py], [vy], [ay], [theta], [w]])
    
    d = v/w_u*(np.cos(theta+w_u*dt)-np.cos(theta))
    e = v/w_u*(-np.sin(theta+w_u*dt)+np.sin(theta))
    #f = -1/dt
    f = dt
    
    # Jacobiano de transición -> Jacobiano con respecto a x_1
    Fx = np.array([[0, 0, 0, 0, 0, 0, d, 0], #df1/dx; f1->ecuacion de posicion de x
                   [0, 0, f, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, e, 0], #df4/dx f1->ecuacion de posicion de y
                   [0, 0, 0, 0, 0, f, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 1]])
    # Jacobiano de control
    Fu = np.array([[ dt, 0],
                   [ 0, dt],
                   [ 0,  0]])
    # Jacobiano de perturbación
    Fe = np.array([[1, 0],
                   [0, 1],
                   [0, 0]])    
    return Xout, Fx, Fu, Fe


def correccion_ekf(x):
    """
    Corrección (medición esperada) a partir del estado actual
    
    """
    
    zpred = np.array([[x[2,0]], [x[5,0]],[x[7,0]]])

    Hx = np.array([[0, 0, 1, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 1, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 1]])
    return zpred, Hx

if __name__ == "__main__": # Inicio del programa principal

    rospy.init_node('nodo_imu_ekf') # Inicializar el nodo
    sub = SuscriptorImu() # Crear l suscriptor
    sub_vel = SuscriptorCmd() # Crear el suscriptor

    # Inicializar broadcaster de TF
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    
    # Creación de una instancia (vacía) del mensaje
    imu_msg = Imu()
    # Creación de una instancia (vacía) del mensaje
    vel_msg = Twist()
    

    # Tiempo de ejecución del bucle (en Hz)
    frecuencia = 40.0
    rate = rospy.Rate(frecuencia)
    dt=1/frecuencia
    
    # Inicialización del estado y de su covarianza
    x = np.array([[-0.3],[0.0],[0.0],[-0.3],[0.0],[0.0],[0.0],[0.0]])   # Valor inicial
    print(x)
    #P = 20.0*np.eye(8)  # Incertidumbre inicial (ejemplo: +-10 -> P=10^2)
    
    
    # Matrices de covarianza
    R = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])            # Perturbación en el incremento de velocidad
    Q = np.diag([0.1, 0.1, 0.1])   # Perturbación en el rango y ángulo

    # Inicialización del estado y su covarianza
    
    P = np.diag([1, 1, 1, 1, 1, 1, 1, 1])**2        # Incertidumbre del estado
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        # Leer el valor actual (del suscriptor)
        imu_msg = sub.get_imu()
        vel_msg = sub_vel.get_vel()
        
        
        # implementación filtro de Kalman
        # Entrada de control conocido
        vx=vel_msg.linear.x
        vy=vel_msg.linear.y
        w = vel_msg.angular.z
        if(w==0):w=+0.0000000000001
        u =np.array([[vx], [vy], [w]])
        
        # print(u)
        # print(x)
        # Obtener las mediciones
        ay = imu_msg.linear_acceleration.y
        ax = imu_msg.linear_acceleration.x
        w = imu_msg.angular_velocity.z
        zmed = np.array([[ax], [ay],[w]]) 
        
        # Predicción
        x, Fx, Fu, Fe = prediccion_ekf(x, u, dt)
        P = Fx@P@Fx.T + R
        # Corrección
        [zpred, Hx] = correccion_ekf(x)
       
        K = P@Hx.T@(np.linalg.inv(Q+Hx@P@Hx.T))
       
        x = x + K@(zmed-zpred)
        P = (np.eye(8)-K@Hx)@P
        
    
        
        pos_x = x[0,0]
        pos_y = x[3,0]
        theta = x[6,0]
        
        # v_x = x[1,0]
        # v_y = x[4,0]
        #w_z = x[7,0]
        
        
        odom_quat =tf.transformations.quaternion_from_euler(0, 0, theta)
        # Hacer broadcast de las posciones del robot con respecto al punto inicial
        odom_broadcaster.sendTransform((pos_x, pos_y, 0.0),
                                        odom_quat,
                                        rospy.Time.now(),
                                        "base_footprint",
                                        "odom")
        
        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(pos_x, pos_y, 0.), Quaternion(*odom_quat))
        
        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(x[1,0], x[4,0], 0), Vector3(0, 0, x[7,0]))
        # odom.twist.twist = Twist(Vector3(v_x, v_y, 0), Vector3(0, 0, w_z))
        #odom.twist.twist.linear.z = w_z
        
        # publish the message
        odom_pub.publish(odom)      
        
        # Esperar
        rate.sleep()
