#!/usr/bin/env python

#Importar rospy 
import rospy

#Librerias basicas
import numpy as np
import cv2

#Librerias custom
from vision_nodes.utils import Vision, pcloud2Rviz

#Importar mensajes
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import message_filters

#Crear callback
def ImageDepthCallback(rgb_msg, depth_msg):

    global josue, cvi, p_pub

    # print("I get sinchronized RGB and Depth")

    #Convertir topicos de imagen a numpy
    img = cvi.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth = cvi.imgmsg_to_cv2(depth_msg)

    # #Mostrar topics
    # cv2.imshow("img_rgb", img)
    # cv2.imshow("depth", depth)
    # cv2.waitKey(10)

    #Calcular nube de puntos
    p_cloud = josue.point_cloud(img, depth)
    
    #Convertir nube de puntos a mensaje de RViz
    p_cloud_msg = pcloud2Rviz(p_cloud, "camera_link")

    #Estampar mensaje
    p_cloud_msg.header.stamp = rospy.Time.now()

    #Publicar mensaje
    p_pub.publish(p_cloud_msg)

def main():

    global josue, cvi, p_pub

    #Inicializar nodo
    rospy.init_node("pcloud_node")

    #Crear CV Bridge
    cvi = CvBridge()

    #Leer parametros de archivo de configuracion
    input_rgb = rospy.get_param("/pcloud_node/topics/rgb", "camera_raw")
    input_depth = rospy.get_param("/pcloud_node/topics/depth", "depth")
    output = rospy.get_param("/pcloud_node/topics/out", "pcloud")

    #2- Parametros de calibracion
    k = rospy.get_param("/pcloud_node/camera/k", [1.0, 1.0, 0.0, 0.0])
    scale = rospy.get_param("/pcloud_node/camera/scale", 1.0)
    d = rospy.get_param("/pcloud_node/camera/d", 1)

    #Crear instacia de vision
    josue = Vision(k[0], k[1], k[2], k[3], scale, d)

    #Crear publisher
    p_pub = rospy.Publisher(output, PointCloud2, queue_size=1)

    #Crear subscirbers por topico
    rgb_sub = message_filters.Subscriber(input_rgb, Image)
    depth_sub = message_filters.Subscriber(input_depth, Image)

    print(input_rgb)
    print(input_depth)

    #Crear sincronizador de tiempo
    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], queue_size=1)

    #Resgitrar Callback
    ts.registerCallback(ImageDepthCallback)

    #Llamar a callbacj
    rospy.spin()


if __name__ == '__main__':
    main()