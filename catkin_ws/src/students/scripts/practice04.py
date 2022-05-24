#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# PRACTICE 4 - PATH FOLLOWING
#
# Instrucciones:
# Escriba el codigo necesario para mover el robot a lo largo de una ruta determinada.
# Considere una base diferencial. Velocidades lineales y angulares maximas
# debe ser 0.8 y 1.0 respectivamente. 
#

import cmath
import rospy
import tf
import math
import numpy
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from custom_msgs.srv import SmoothPath, SmoothPathRequest
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point

NAME = "GONZALEZ_JIMENEZ_"

pub_goal_reached = None
pub_cmd_vel = None
loop        = None
listener    = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    
    # TODO:
    # Implementar la ley de control dada por:
    #
    # v = v_max*math.exp(-error_a*error_a/alpha)
    # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    #
     # v_max, w_max, alpha y beta, son constantes de ajuste.
    alpha, beta = 0.1, 0.3
    v_max, w_max = 0.8, 0.6
    # donde error_a es el angulo de error y
    # v y w son las velocidades lineales y angulares tomadas como seniales de entrada

    # error_a = ag - a
    error_a = math.atan2((goal_y - robot_y),(goal_x - robot_x)) - robot_a

    # Normaliza angulo de error a valores en el intervalo (-pi,pi]
    if error_a < -math.pi:   # Si el angulo negativo sale de rango por la izq
        error_a = error_a + 2*math.pi
    if error_a > math.pi:
        error_a = error_a - 2*math.pi

    v = v_max*math.exp(-error_a*error_a/alpha)
    w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)

    # Almacena la v y la w resultantes en el mensaje Twist cmd_vel
    cmd_vel = Twist()
    cmd_vel.linear.x = v
    cmd_vel.angular.z = w
    
    return cmd_vel

def follow_path(path):
    #
    # TODO:
    # Use la funcion de calcular_control para mover el robot a lo largo de la ruta.
    # La ruta se da como una secuencia de puntos [[x0,y0], [x1,y1], ..., [xn,yn]]
    # El publicador del mensaje Twist ya esta declarado como 'pub_cmd_vel'
    #
    # Puede usar los siguientes pasos para realizar el seguimiento de la ruta:
    #
    # Si el error local es menor a 0.3 (puedes cambiar esta constante
    # Cambiar el punto de destino local al siguiente punto en la ruta
    tol = 0.3
    
    e_local = float(numpy.inf )
    msg_vel = Twist()
    n = 0
    # Establecer el punto de destino local como el primer punto de la ruta
    for x_g_local, y_g_local in path:
        [robot_x, robot_y, robot_a] = get_robot_pose(listener)
        e_local = math.sqrt((x_g_local - robot_x)**2 + (y_g_local - robot_y)**2)
        # WHILE local error > tol and not rospy.is_shutdown(), Esto mantiene al programa al tanto de seniales como Ctrl+C
        while e_local > tol and  not rospy.is_shutdown():
            # Obtener la posicion del robot:
            [robot_x, robot_y, robot_a] = get_robot_pose(listener)      
            # Calcule el error local como la magnitud del vector desde la posicion del robot hasta el punto objetivo local
            e_local = math.sqrt((x_g_local - robot_x)**2 + (y_g_local - robot_y)**2)
            print("error local: ", e_local)
            # Calcule el error global como la magnitud del vector desde la posicion del robot hasta el punto objetivo global
            x_g_global, y_g_global = path[-1]
            e_global = math.sqrt((x_g_global - robot_x)**2 + (y_g_global - robot_y)**2)
            # Calcular las seniales de control v y w y publicar el mensaje correspondiente
            # el metod "calculate_control" retorna un objeto Twist
            msg_vel = calculate_control(robot_x, robot_y, robot_a, x_g_local, y_g_local)
            pub_cmd_vel.publish(msg_vel)
            print("***********Iteracion ", n)
            n += 1
            loop.sleep() #Esto es importante para evitar un consumo excesivo de tiempo de procesamiento
    print("Termino la ruta*********")
    # Publicar un  'True' usando un publicador pub_goal_reached
    pub_goal_reached.publish(True)
    # Enviar velocidades cero (de lo contrario, el robot seguira moviendose despues de llegar al ultimo punto)
    msg_vel.linear.x = 0
    msg_vel.angular.z = 0
    pub_cmd_vel.publish(msg_vel)
    return
    
    # Respuesta a la peticion de obtener la posicon y orientacion del robot
def callback_global_goal(msg):
    print("Calculating path from robot pose to " + str([msg.pose.position.x, msg.pose.position.y]))
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    req = GetPlanRequest(goal=PoseStamped(pose=msg.pose))
    req.start.pose.position = Point(x=robot_x, y=robot_y)
    path = rospy.ServiceProxy('/path_planning/a_star_search', GetPlan)(req).plan
    path = rospy.ServiceProxy('/path_planning/smooth_path',SmoothPath)(SmoothPathRequest(path=path)).smooth_path
    print("Following path with " + str(len(path.poses)) + " points...")
    follow_path([[p.pose.position.x, p.pose.position.y] for p in path.poses])
    print("Global goal point reached")

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]

def main():
    global pub_cmd_vel, pub_goal_reached, loop, listener
    print("PRACTICE 04 - " + NAME)
    rospy.init_node("practice04")   # Inicia el nodo P4
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_global_goal)   # Crea un suscriptor a mensajes tipo PoseStamped
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)     # crea un publicador de mensajes tipo Twist
    pub_goal_reached = rospy.Publisher('/navigation/goal_reached', Bool, queue_size=10)  # Crea un publicador de mensajes tipo Bool
    listener = tf.TransformListener()  # Crea un objeto de la clase tf     
    loop = rospy.Rate(10)       
    print("Waiting for service for path planning...")
    rospy.wait_for_service('/path_planning/a_star_search')
    print("Service for path planning is now available.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
