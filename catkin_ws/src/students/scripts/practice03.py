#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# PRACTICE 3 - PATH SMOOTHING BY GRADIENT DESCEND
#
# Instructions:
# Write the code necessary to smooth a path using the gradient descend algorithm.
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import numpy
import heapq
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from custom_msgs.srv import SmoothPath
from custom_msgs.srv import SmoothPathResponse

NAME = "GONZALEZ_JIMENEZ"

msg_smooth_path = Path()

def smooth_path(Q, alpha, beta):
    print("Smoothing path with params: " + str([alpha,beta]))
    #
    # TODO:
    # Escriba el codigo para suavizar la ruta Q, usando el algoritmo de descenso de gradiente,
    # y devuelve una nueva ruta suavizada P.
    # La ruta se compone de un conjunto de puntos [x,y] de la siguiente manera:
    # [[x0,y0], [x1,y1], ..., [xn,ym]].
    # El camino suavizado debe tener la misma forma.
    # Devuelve el camino suavizado. 

    P = numpy.copy(Q)  
    #print("P: ",P)                             # Ruta inicial P
    tol     = 0.00001                               # Tolerancia de la magnitud del gradiente
    nabla   = numpy.full(Q.shape, float("inf"))     # Gradiente de la funcion J
    epsilon = 0.1                                   # Tamanio del paso
    N = len(P)                                      # Dimension del gradiente
    #print("Dim N = ", N)
    
    rospy.loginfo("Inicializacion finalizada")
    
    while numpy.linalg.norm(nabla) > tol: # Verifica el tamanio del gradiente
        i = 1
        # gradiente en J(p)
        while i <= N-2:  # Calcula el gradiente en cada componente i del vector nabla excepto en las orillas
            elemento_i = alpha*(P[i]-P[i-1]) - alpha*(P[i+1] - P[i]) + beta*(P[i]-Q[i])
            #print("elemento_i", elemento_i)
            # Agrega la i-esima componente de nabla comenzando en indice 1
            nabla[i] = elemento_i
            i += 1
        
        #Primer y ultimo elemento del gradiente se hacen cero
        nabla[0] = [0,0]
        nabla[N-1] = [0,0]

        # Actualizacion de puntos P
        # Resta sucecivamente a P el gradiente por el tamanio del paso, 
        # lleva a cada punto de P a su valor 
        P = P - epsilon*nabla
        #print("nabla_magnitud: ", numpy.linalg.norm(nabla))
    rospy.loginfo("Retornando ruta suavizada")

    return P



def callback_smooth_path(req):
    alpha = rospy.get_param('/path_planning/smoothing_alpha')
    beta  = rospy.get_param('/path_planning/smoothing_beta' )
    P = smooth_path(numpy.asarray([[p.pose.position.x, p.pose.position.y] for p in req.path.poses]), alpha, beta)
    msg_smooth_path.poses = []
    for i in range(len(req.path.poses)):
        msg_smooth_path.poses.append(PoseStamped(pose=Pose(position=Point(x=P[i,0],y=P[i,1]))))
    return SmoothPathResponse(smooth_path=msg_smooth_path)

def main():
    print ("PRACTICE 03 - " + NAME)
    rospy.init_node("practice03", anonymous=True)
    rospy.Service('/path_planning/smooth_path', SmoothPath, callback_smooth_path)
    pub_path = rospy.Publisher('/path_planning/smooth_path', Path, queue_size=10)
    loop = rospy.Rate(1)
    msg_smooth_path.header.frame_id = "map"
    while not rospy.is_shutdown():
        pub_path.publish(msg_smooth_path)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
