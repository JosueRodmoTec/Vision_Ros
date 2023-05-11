#Additional libs
import rospy
import numpy as np
import struct

#ROS utils
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

#Utilidades
from sensor_msgs import point_cloud2

#Convertir nube de puntos a numpy
def pcloud2numpy(p_cloud):
  #Extraer nube de puntos
  points = point_cloud2.read_points(p_cloud, skip_nans=True, field_names=("x", "y", "z"))

  #Nube de puntos a numpy
  points = np.array(list(points))

  return points

#Functions
def hello():

  print("Hello!")
  
class Vision():

  #Constructos
  def __init__(self, fx, fy, cx, cy, scale, d):

    #Definir matriz de parametros intrinsecos
    self.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    self.K_inv = np.linalg.inv(self.K)

    #Escala de nube de puntos
    self.scale = scale

    #Densidad de la nube
    self.d = d

    #Rotacion camara-mundo
    theta = -np.pi/2
    self.R = np.array([[1.0, 0.0, 0.0], [0.0, np.cos(theta), -np.sin(theta)], [0.0, np.sin(theta), np.cos(theta)]])

  #Metodo 1: Calcular nube de puntos
  ## Input:
  ##    img: Imagen RGB
  ##    depth: Mapa de profundidad
  ##    d: Densidad de la nube
  ##Output:
  ##    p_cloud: Lista con puntos en 3D
  def point_cloud(self, img, depth):

    #Inicializar nube de puntos
    p_cloud = []
    
    #Ciclo de filas
    for i in range(img.shape[0]):
      #Densidad
      if(i%self.d == 0):
        for j in range(img.shape[1]):

          #Densidad
          if(j%self.d == 0):

            #Crear vecto de puntos en 2D
            p_2d = np.array([[j], [i], [1]])

            #Obtener z de mapa de
            z = self.scale*depth[i, j]
            if(z > 0):

              #Calcular punto de 3D
              p_3d = z*np.dot(self.K_inv, p_2d)

              #Camara - Rviz
              p_3d = np.dot(self.R, p_3d)

              #Extraer color de la imagen (imagen BHR)
              b = img[i, j, 0]
              g = img[i, j, 1]
              r = img[i, j, 2]
              a = 255

              #Contruis punto
              rgb = struct.unpack("I", struct.pack('BBBB', b, g, r, a))[0]
              pt = [p_3d[0,0], p_3d[1,0], p_3d[2,0], rgb]
              p_cloud.append(pt)
    
    #Regresa nube de punto
    return p_cloud

#Metodo 2: Convertir lista a nube de puntos
def pcloud2Rviz(p_cloud, frame):

  header = Header()
  header.frame_id = frame

  #Crear campos
  fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1)] #FLOAT32

  #Crear nube de puntos
  pc_msg = point_cloud2.create_cloud(header, fields, p_cloud)

  return pc_msg

#Mapas de ocupacion
class OccGrid():

  #Constructor
  def __init__(self, x_g, y_g, d_g, z_min, z_max, z_floor, p_thresh, p_occ, p_free):

    #Guardar atributos

    #a) Parametros de la gradilla
    self.x_g = x_g 
    self.y_g = y_g
    self.d_g = d_g
    self.z_min = z_min
    self.z_max = z_max
    self.z_floor = z_floor

    #b) Parametros del algoritmo
    self.p_thresh = p_thresh #(cota de obstaculo)
    self.p_occ = p_occ #(Aporte de obstaculo)
    self.p_free = p_free #(Aporte de espacio libre)

    #Inicializar arreglos auxiliares
    self.map = np.zeros([self.x_g, self.y_g])

    #Realizacion de probabilidades
    self.l_occ = np.log(self.p_occ/self.p_free)
    self.l_free = np.log(self.p_free/self.p_occ)

  
  #Metodos de instacia
  def update(self, p_cloud):

    #Analizar nube de puntos
    for i in range(p_cloud.shape[0]):

      #Descomponer punto
      x = p_cloud[i, 0]
      y = p_cloud[i, 1]
      z = p_cloud[i, 2]

      #Verificar que punto se encuentre en region de analisis
      if (z < self.z_max and z > self.z_min):

        # Mapear puntos con odometria (camara -> mundo)
        #Algun dia volver ...

        #Obtener coordenadas del punto respecto a grandilla
        a, b = self.get_index(x, y)

        #Verificar que punto se encuentre dentro de la grandilla
        if(a > 0 and a < self.map.shape[0] and b > 0 and b < self.map.shape[1]):

          #Actualizar como obstaculo
          self.map[a, b] += self.l_occ
      
      if(z < self.z_floor):

        #Obtener coordenadas del punto respecto a la grandilla
        a, b = self.get_index(x, y)

        #Verificar que punto se encuentre dentro de la grandilla
        if(a > 0 and a < self.map.shape[0] and b > 0 and b < self.map.shape[1]):

          #Actualizar como obstaculo
          self.map[a, b] += self.l_free

  #Calcular posicion en gradilla
  def get_index(self, x, y):

    #Calcular posicion
    a = int(np.ceil(x / self.d_g) + 0.5 * self.map.shape[0])
    b = int(np.ceil(y / self.d_g) + 0.5 * self.map.shape[1])

    return a, b
  
  #Obtener mapa de ocupacion
  def get_map(self):

    occ_map = 1.0 - 1.0/(1 + np.exp(self.map))

    return occ_map
  #Gernerar mapa en rviz
  def get_rviz(self, occ_map, ros_time):
    #inicializar arreglo de marcadores
    msg_grid = MarkerArray()

    #Analizar mapa de ocupacion
    for i in range(occ_map.shape[0]):
      for j in range(occ_map.shape[1]):

        #Inicializar un marcador
        marker = Marker()

        #Inicializar marcador
        marker.header.frame_id = "world"
        marker.header.stamp = ros_time
        marker.type = 1
        marker.id = i*occ_map.shape[1] + j

        #Definir exala del amrcador
        marker.scale.x = self.d_g
        marker.scale.y = self.d_g
        marker.scale.z = 0.2*self.d_g

        #Posicion del marcador
        marker.pose.position.x = self.d_g*(i -0.5*self.map.shape[0])
        marker.pose.position.y = self.d_g*(j -0.5*self.map.shape[1])
        marker.pose.position.z = 0.0

        #Definir color de marcador
        #Obstaculo
        if(occ_map[i, j] > self.p_occ):
          marker.color.r = 0.0
          marker.color.g = 0.0
          marker.color.b = 0.0
          marker.color.a = 0.5

        #Punto desconocido
        if(occ_map[i, j] > self.p_free and occ_map[i, j] < self.p_occ):
          marker.color.r = 0.5
          marker.color.g = 0.5
          marker.color.b = 0.5
          marker.color.a = 0.5
        
        #Zona libre
        if(occ_map[i, j] < self.p_free):
          marker.color.r = 1.0
          marker.color.g = 1.0
          marker.color.b = 1.0
          marker.color.a = 0.5

        #Agregar marcador a list
        msg_grid.markers.append(marker)

    return msg_grid

        



  #Metodos estaticos
  def st_method():

    pass