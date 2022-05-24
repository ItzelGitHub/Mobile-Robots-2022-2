#!/usr/bin/env python
#
# MOBILE ROBOTS - UNAM, FI, 2022-2
# PRACTICE 08 - INVERSE KINEMATICS
#
# Instrucciones:
# Calcule la cinematica inversa para ambos manipuladores (izquierdo y derecho) dada la
# Descripciones URDF y una configuracion deseada. Resuelve la cinematica inversa usando
# el metodo de Newton-Raphson para encontrar raices.
# Modify only sections marked with the 'TODO' comment
# http://docs.ros.org/en/jade/api/tf/html/python/transformations.html

from __future__ import print_function
import math
from turtle import shape
import rospy
import tf
import tf.transformations as tft
import numpy 
import urdf_parser_py.urdf
from geometry_msgs.msg import PointStamped
from custom_msgs.srv import *

NAME = " GONZALEZ_JIMENEZ_ITZEL"

def get_model_info():
    global joints, transforms
    robot_model = urdf_parser_py.urdf.URDF.from_parameter_server()
    joints = {'left': [None for i in range(8)], 'right': [None for i in range(8)]}
    transforms = {'left':[], 'right':[]}
    for joint in robot_model.joints:
        for i in range(1,8):
            joints['left' ][i-1] = joint if joint.name == ('la_'+str(i)+'_joint') else joints['left' ][i-1]
            joints['right'][i-1] = joint if joint.name == ('ra_'+str(i)+'_joint') else joints['right'][i-1]
        joints['left' ][7] = joint if joint.name == 'la_grip_center_joint' else joints['left' ][7]
        joints['right'][7] = joint if joint.name == 'ra_grip_center_joint' else joints['right'][7]
    for joint in joints['left']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['left'].append(tft.concatenate_matrices(T,R))
    for joint in joints['right']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['right'].append(tft.concatenate_matrices(T,R))
    
def forward_kinematics(q, Ti, Wi):
    # q: arreglo con los valores de las 7 articulaciones 
    # Ti: lista con 7 matrices de transformacion cuando qi esta en la posicion ZERO
    # Wi: Lista que contiene los ejes de rotacion de q1,q2,q3,..,q7,none
    # TODO:
    # Calcule la cinematica directa dado un conjunto de 7 angulos 'q'
    # Ti[7] es la transformacion homogenea final desde el centro del gripper hasta la articulacion 7.
    #print("*Entrando a Cinematica Directa............")
    R1 = tft.rotation_matrix(q[0],Wi[0])
    R2 = tft.rotation_matrix(q[1],Wi[1])
    R3 = tft.rotation_matrix(q[2],Wi[2])
    R4 = tft.rotation_matrix(q[3],Wi[3])
    R5 = tft.rotation_matrix(q[4],Wi[4])
    R6 = tft.rotation_matrix(q[5],Wi[5])
    R7 = tft.rotation_matrix(q[6],Wi[6])
    R = [R1,R2,R3,R4,R5,R6,R7]  

    H  = tft.identity_matrix()
    for i in range(len(q)):
        # Relacionando cada frame con su eje de rotacion y distancia a la base
        H  = tft.concatenate_matrices(H, Ti[i],R[i])

    H07  = tft.concatenate_matrices(H, Ti[7]) # Transformacion entre griper y hombro(base)
    x = H07[0,3]
    y = H07[1,3]
    z = H07[2,3]          
    R,P,Y = tft.euler_from_matrix(H) 

    #print("POSICION: x:",x,"y:",y,"z:",z)
    #print("ORIENTACION RAD: ROW:",R,",PITCH:",P,",YAW:",Y)
    #print("ORIENTACION GRAD: ROW:",numpy.degrees(R),",PITCH:",numpy.degrees(P),",YAW:",numpy.degrees(Y))
    #print("Cinematica Directa Obtenida.....\n")
    return numpy.asarray([x,y,z,R,P,Y])


def jacobian(q, Ti, Wi):
    #print("Calculando el jacobiano\n")
    delta_q = 0.00001
    #
    # TODO:
    # Calcular el jacobiano dada una descripcion cinematica Ti y Wi
    # donde:
    # Ti es la matriz de transformacion homogenea desde el sistema i hasta el sistema i-1 
    # cuando la articulacion i esta en la posicion ZERO
    # Wi es el eje de rotacion de la i-esima articulacion
    # Use la aproximacion numerica:   f'(x) = (f(x+delta) - f(x-delta))/(2*delta)
    # Cada columna es la variacion con respecto a una misma articulacion (angulo)

    # Puedes seguir los siguientes pasos:
    #     J = matrix of 6x7 full of zeros
    #     q_next = [q1+delta       q2        q3   ....     q7
    #                  q1       q2+delta     q3   ....     q7
    #                              ....
    #                  q1          q2        q3   ....   q7+delta]
    #     q_prev = [q1-delta       q2        q3   ....     q7
    #                  q1       q2-delta     q3   ....     q7
    #                              ....
    #                  q1          q2        q3   ....   q7-delta]
    #     FOR i = 1,..,7:
    #           i-th column of J = ( FK(i-th row of q_next) - FK(i-th row of q_prev) ) / (2*delta_q)
    #     RETURN J
    #     
    J = numpy.asarray([[0.0 for a in q] for i in range(6)])            # J 6x7 full of zeros
    
    # COnvertimos una estructura de datos en matriz 7x7 y se suma y resta delta a los elementos de la
    # diagonal principal en cada arreglo qn y qp
    qn = numpy.asarray([q,]*len(q)) + delta_q*numpy.identity(len(q))   # q_next as indicated above
    qp = numpy.asarray([q,]*len(q)) - delta_q*numpy.identity(len(q))   # q_prev as indicated above

    # Calculo de f'(x) = (f(x+delta) - f(x-delta))/(2*delta)
    i = 0
    for i in range(len(q)):    # Para cada columna de J
        # Usamos la cinematica directa
        # i-th column of J = ( FK(i-th row of q_next) - FK(i-th row of q_prev) ) / (2*delta_q)
        # En cada ciclo pasamos la articulacion qi
        x,y,z,R,P,Y = (forward_kinematics(qn[i], Ti, Wi) - forward_kinematics(qp[i], Ti, Wi)) / delta_q
        J[:,i] = x,y,z,R,P,Y 
    
    #rospy.loginfo("Jacobiano calculado.....")
    return J



def inverse_kinematics_xyzrpy(x, y, z, roll, pitch, yaw, Ti, Wi,q = numpy.asarray([-0.5, 0.6, 0.3, 2.0, 0.3, 0.2, 0.3])):
    #print("Calculando la Cinematica Inversa........")
    # Configuracion en el espacio cartesiano deseada 
    
    pd = numpy.asarray([x,y,z,roll,pitch,yaw])  
    tolerance = 0.00001
    max_iterations = 40
    iterations = 0
    
    # TODO:
    # Resolver el problema IK dada una descripcion cinematica (Ti, Wi) y una configuracian deseada.
    # donde:
    # Ti son las transformaciones homogeneas del marco i al marco i-1 cuando la articulacion i esta en la posicion cero
    # Wi es el eje de rotacion de la i-esima articulacion
    # Use el metodo de Newton-Raphson para encontrar raices. (Encuentre las raices de la ecuacion FK(q) - pd = 0)
    # Puedes hacer los siguientes pasos:

    # Establezca una suposicion inicial para las articulaciones 'q'. 
     
    # Calcule la cinematica directa 'p' llamando a la funcion correspondiente
    p = forward_kinematics(q, Ti, Wi)
    # Calcular error. Asegurese de que los angulos de orientacion del error esten en [-pi,pi]
    error = p - pd
    error[3:6] = (error[3:6] + math.pi)%(2*math.pi) - math.pi
    n_error = numpy.linalg.norm(error)
    #print("q antes",q)
    # MIENTRAS |error| > TOL e iteraciones < iteraciones maximas:
    while (iterations < max_iterations) and (n_error > tolerance):
        # Calcular jacobiano
        J = jacobian(q, Ti, Wi)
        # Actualizar la estimacion de q 
        q = q - numpy.dot(numpy.linalg.pinv(J), error)
        # Asegurese de que todos los angulos q esten en [-pi,pi]
        q = (q  + math.pi)%(2*math.pi) - math.pi
        # Recalcular cinematica directa p
        p = forward_kinematics(q, Ti, Wi)
        # Vuelva a calcular el error y asegurese de que los angulos esten en [-pi, pi]
        error = p - pd
        error[3:6] = (error[3:6] + math.pi)%(2*math.pi) - math.pi

        n_error = numpy.linalg.norm(error)
        # Incrementar iteraciones
        iterations += 1
        print("Iteraciones: ",iterations)

    # Retorno calculado q si no se excedieron las iteraciones maximas
    # De lo contrario, devuelve Ninguno
    #print("q: ",q)

    if iterations <=40:
        print("Cinematica Inversa calculada ........")
        print("error: ",n_error)
        print("EN GRADOS:",numpy.degrees(q))
        return q
    
    else:
        print("Se excedio el numero de iteraciones maximas, return es none")
    
    print("return es none")
    return None

def callback_la_ik_for_pose(req):
    global transforms, joints
    Ti = transforms['left']                               
    Wi = [joints['left'][i].axis for i in range(len(joints['left']))]  
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, Ti, Wi)
    if q is None:
        return None
    resp = InverseKinematicsResponse()
    [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_ra_ik_for_pose(req):
    global transforms, joints
    Ti = transforms['right']                               
    Wi = [joints['right'][i].axis for i in range(len(joints['right']))]  
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, Ti, Wi)
    if q is None:
        return False
    resp = InverseKinematicsResponse()
    [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_la_fk(req):
    # req = q1,q2,q3,q4,q5,q6,q7
    global transforms, joints
    # Asignamos a Ti los valores 
    Ti = transforms['left']                               
    Wi = [joints['left'][i].axis for i in range(len(joints['left']))]  
    x = forward_kinematics([req.q1, req.q2, req.q3, req.q4, req.q5, req.q6, req.q7], Ti, Wi)
    resp = ForwardKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def callback_ra_fk(req):
    global transforms, joints
    Ti = transforms['right']                               
    Wi = [joints['right'][i].axis for i in range(len(joints['right']))]  
    x = forward_kinematics([req.q1, req.q2, req.q3, req.q4, req.q5, req.q6, req.q7], Ti, Wi)
    resp = ForwardKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def main():
    print("PRACTICE 08" + NAME)
    rospy.init_node("ik_geometric")
    print("Obteniendo informacion del modelo......")
    get_model_info()
    # Se crean los servicios de cinematica inversa y directa de brazo izquierdo y derecho
    rospy.Service("/manipulation/la_inverse_kinematics", InverseKinematics, callback_la_ik_for_pose)
    rospy.Service("/manipulation/ra_inverse_kinematics", InverseKinematics, callback_ra_ik_for_pose)
    rospy.Service("/manipulation/la_direct_kinematics", ForwardKinematics, callback_la_fk)
    rospy.Service("/manipulation/ra_direct_kinematics", ForwardKinematics, callback_ra_fk)

    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
