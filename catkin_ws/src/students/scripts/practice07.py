#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# PRACTICE 7 - COLOR SEGMENTATION
#
# Instrucciones:
# Complete el codigo para estimar la posicion de un objeto
# dada una nube de puntos coloreada usando segmentacion de color.
#

import numpy
import cv2
import ros_numpy
import rospy
import math
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from custom_msgs.srv import FindObject, FindObjectResponse

NAME = "GONZALEZ_JIMENEZ_ITZEL"

def segment_by_color(img_bgr, points, obj_name):
    # Imagen de 480x640 pixeles
    # TODO:
    # - Asigna limites de color inferior y superior en hsv segun el objeto solicitado:, 
    #   cada valor del vector corresponde a cada valor H, S y V
    if (obj_name == 'pringles'): 
        min_valor = numpy.array([25, 50, 50]) 
        max_valor = numpy.array([35, 255, 255])
    else: #[10,200, 50] - [20, 255, 255]
        min_valor = numpy.array([10,200, 50]) 
        max_valor = numpy.array([20, 255, 255])
    # - Cambia el espacio de color de BGR a HSV.
    img_hsv = cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)
    
    # - Determinar los pixeles cuyo color esta en el rango de color seleccionado.
    #   Deteccion del color: creamos una mascara que contiene solo los colores definidos en los limites
    #   Regresa imagen binaria: pixeles blancos si entro en el rango, sino pixeles negros.
    img_bin = cv2.inRange(img_hsv,min_valor, max_valor)
    #cv2.imshow("binario",img_bin)

    #   cv2.findNonZero():Entra una imagen binaria y Devuelve la lista de ubicaciones de pixeles distintos de cero.
    img_non_zero = cv2.findNonZero(img_bin)
    # - Calcule el centroide de todos los pixeles en el rango de color dado (posicion de la bola).
    #   cv2.mean():Calcula un promedio (media) de los elementos de la matriz.
    centroide_pixel = cv2.mean(img_non_zero)
    centroide_r, centroide_c = centroide_pixel[0], centroide_pixel[1]
    
    # - Calcular el centroide de la region segmentada en el espacio cartesiano
    #   usando la nube de puntos 'points'. Utilice la notacion de matriz numpy para procesar los datos de la nube de puntos.
    #   Ejemplo: 'points[240,320][1]' obtiene el valor 'y' del punto correspondiente a
    #   el pixel en el centro de la imagen.
    x, y, z = 0,0,0
    for cord in img_non_zero:
        c,r = cord[0]           # columna , renglon
        print(cord)
    
        x += points[r,c][0] # accedemos a la posicion actual (r,c) en la imagen y al valor del 1r elemento (coordenada x)
        y += points[r,c][1]
        z += points[r,c][2]
        print("Valores que va obteniendo de Z: ",points[r,c][2])

    centroide_x = x/len(img_non_zero)
    centroide_y = y/len(img_non_zero)
    centroide_z = z/len(img_non_zero)

    print("centroide xyz: ",centroide_x,centroide_y,centroide_z)
    print("cr, cc",centroide_r,centroide_c)
    print("Centroide calculado")
    print("***********")

    #cv2.waitKey(10)

    return[centroide_r,centroide_c, centroide_x, centroide_y, centroide_z]


def callback_find_object(req):
    # Este metodo atiende la respuesta del servicio find_object
    global pub_point, img_bgr
    print("Intentando encontrar el objeto: " + req.name)
    # Funcion defindida por usuario si o no
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(req.cloud)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    img_bgr = cv2.merge((numpy.asarray(b,dtype='uint8'),numpy.asarray(g,dtype='uint8'),numpy.asarray(r,dtype='uint8')))
    [r, c, x, y, z] = segment_by_color(img_bgr, arr, req.name)
    hdr = Header(frame_id='kinect_link', stamp=rospy.Time.now())
    pub_point.publish(PointStamped(header=hdr, point=Point(x=x, y=y, z=z)))
    cv2.circle(img_bgr, (int(r), int(c)), 20, [0, 255, 0], thickness=3)
    resp = FindObjectResponse()
    resp.x, resp.y, resp.z = x, y, z
    return resp

def main():
    global pub_point, img_bgr
    print("PRACTICE 07 - " + NAME)
    rospy.init_node("color_segmentation")
    # Suscripcion al servicio find_object
    rospy.Service("/vision/find_object", FindObject, callback_find_object)
    # Crea un publicador 
    pub_point = rospy.Publisher('/detected_object', PointStamped, queue_size=10)
    # Crea una imagen bgr
    img_bgr = numpy.zeros((480, 640, 3), numpy.uint8)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        cv2.imshow("Color Segmentation", img_bgr)
        cv2.waitKey(1)
        loop.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
