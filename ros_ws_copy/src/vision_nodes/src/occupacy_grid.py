#!/usr/bin/env python

#Importar rospy 
import rospy

#Librerias basicas
import numpy as np
import cv2

#Librerias custom
from vision_nodes.utils import Vision, OccGrid, pcloud2numpy

#Importar mensajes
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

#Utilidades
from sensor_msgs import point_cloud2

#Matplotlib
import matplotlib.pyplot as plt

#Subscriber
def PCloudCallback(point_cloud):
    global edwin, display, rviz, marker_pub

    #print("Recibi nube de puntos!! oh shit!")
    
    #Extrar nube de puntos
    points = pcloud2numpy(point_cloud)

    # print(points.shape)

    #Actualizar mapa de ocupacion
    edwin.update(points)

    #Obtener mapa de ocupacion
    occ_map = edwin.get_map()

    #Mostrar mapa de ocupacion
    if(display):
        plt.ion()
        plt.figure(1)
        plt.clf()
        plt.imshow(occ_map, 'Greys')
        plt.pause(0.005)
        print(occ_map)
    
    #Publicar mapa de Rviz
    if(rviz):

        #Generar arreglo de rviz
        rviz_msg = edwin.get_rviz(occ_map, rospy.Time.now())

        #Publicar mensaje
        marker_pub.publish(rviz_msg)


def main():
    global edwin, display, rviz, marker_pub
    #Inicializar nodo
    rospy.init_node("occupacy_grid")

    #Leer parametros de archivo de configuracion


    #1- Topicos de entrada
    input_cloud = rospy.get_param("/occupacy_grid/topics/cloud", "/pcloud")

    #2 - Parametrps de la grandilla
    x_grid = rospy.get_param("/occupacy_grid/grid/x", 10)
    y_grid = rospy.get_param("/occupacy_grid/grid/y", 10)
    d_grid = rospy.get_param("/occupacy_grid/grid/dim", 1.0)

    #3 - Parametros del algoritmo
    p_thresh = rospy.get_param("/occupacy_grid/params/p_thresh", 0.8)
    p_occ = rospy.get_param("/occupacy_grid/params/p_occ", 0.66)
    p_free = rospy.get_param("/occupacy_grid/params/p_free", 0.34)
    z_min = rospy.get_param("/occupacy_grid/params/z_min", 0.0)
    z_max = rospy.get_param("/occupacy_grid/params/z_max", 0.5)
    z_floor = rospy.get_param("/occupacy_grid/params/z_floor", -0.8)

    display = rospy.get_param("/occupacy_grid/interfeace/display", True)
    rviz = rospy.get_param("/occupacy_grid/interfeace/rviz", True)

    #Crear instacia de mapa de ocupacion
    edwin = OccGrid(x_grid, y_grid, d_grid, z_min, z_max, z_floor, p_thresh, p_occ, p_free)

    #Crear subscribers
    cloud_sub = rospy.Subscriber(input_cloud, PointCloud2, PCloudCallback, queue_size=1)

    #Crear publishers
    marker_pub = rospy.Publisher("/occupacy_grid/rviz_grid", MarkerArray, queue_size = 1)

    #Llamar a callback
    rospy.spin()


if __name__ == '__main__':
    main()