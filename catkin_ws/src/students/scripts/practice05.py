#!/usr/bin/env python

#

# MOBILE ROBOTS - FI-UNAM, 2022-2

# PRACTICA 5 - EVASION DE OBSTACULOS POR CAMPOS POTENCIALES

#

# Instructions:

# Complete el codigo para implementar la evitacion de obstaculos por campos potenciales

# usando la tecnica de campos atractivos y repulsivos.

# Sintonice las constantes alfa y beta para obtener un movimiento suave. 

#



import rospy

import tf

import math

from std_msgs.msg import Header

from geometry_msgs.msg import Twist

from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Point

from visualization_msgs.msg import Marker

from sensor_msgs.msg import LaserScan



NAME = "GONZALEZ_JIMENEZ"



listener    = None

pub_cmd_vel = None

pub_markers = None



def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):

    cmd_vel = Twist()

    #

    # TODO:

    # Implementar la ley de control dada por:

    #

    # v = v_max*math.exp(-error_a*error_a/alpha)

    # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)

    #

     # v_max, w_max, alpha y beta, son constantes de ajuste.

    alpha, beta = 0.4, 0.3

    v_max, w_max = 0.6, 0.6

    # donde error_a es el angulo de error y

    # v y w son las velocidades lineales y angulares tomadas como seniales de entrada



    # error_a = ag - a  VERIFICAR UNIDADES*********************

    error_a = math.atan2((goal_y - robot_y),(goal_x - robot_x)) - robot_a



    # Normaliza angulo de error a valores en el intervalo (-pi,pi]

    if error_a < -math.pi:   # Si el angulo negativo sale de rango por la izq

        error_a = error_a + 2*math.pi

    if error_a > math.pi:

        error_a = error_a - 2*math.pi



    v = v_max*math.exp(-error_a*error_a/alpha)

    w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)



    # Almacena la v y la w resultantes en el mensaje Twist cmd_vel

    cmd_vel_msg = Twist()

    cmd_vel_msg.linear.x = v

    cmd_vel_msg.angular.z = w  



    return cmd_vel_msg



def attraction_force(robot_x, robot_y, goal_x, goal_y):

    #

    # TODO: REVISAR****************************

    # Calcule la fuerza de atraccion, dadas las posiciones del robot y del objetivo.

    alfa = 0.3

    dx, dy = robot_x - goal_x, robot_y - goal_y

    mag = (dx**2 + dy**2)**0.5

    force_x, force_y = alfa*(dx/mag), alfa*(dy/mag) 

    # Devuelve una tupla de la forma [force_x, force_y]

    # donde force_x y force_y son los componentes X e Y

    # de la fuerza de atraccion resultante w.r.t. mapa.

    

    return [force_x, force_y]



def rejection_force(robot_x, robot_y, robot_a, laser_readings):

    #

    # TODO:

    # Calcule la fuerza de rechazo total dada por el promedio

    # de las fuerzas de rechazo causadas por cada lectura de laser.

    n = len(laser_readings)

    betha = 1.6

    d0 = 1  # 1 metro

    frx, fry = 0, 0  # Campo repulsivo en x y y

    # laser_readings es una matriz donde cada elemento es una tupla [distancia, angulo]

    # ambos medidos w.r.t. con respecto al sistema coordenado del robot.

    # Consulte las notas de clase para conocer las ecuaciones para calcular las fuerzas de rechazo.

    for d, theta in laser_readings:  # Angulos y distancias dadas por el sensor

        if d >= d0:     # Los obstaculos que estan demasiado lejos se ignoran

            continue



        mag = betha*((1/d - 1/d0)**0.5)

        frx = frx + mag*math.cos(theta+robot_a)  # x=mag*cos(theta)

        fry = fry + mag*math.sin(theta+robot_a)  # y=mag*sen(theta)

    force_x , force_y = frx/n, fry/n 

    # Devuelve una tupla de la forma [force_x, force_y]

    # donde force_x y force_y son los componentes X e Y

    # de la fuerza de rechazo resultante w.r.t. mapa.

    

    return [force_x, force_y]



def callback_pot_fields_goal(msg):

    # Esta funcion esta suscrita a /move_base_simple/goal y recibe la informacion publicada.

    # espera mensajes de objetivo (geometry_msgs/Pose) sobre el topico move_base_simple/goal.

    goal_x = msg.pose.position.x

    goal_y = msg.pose.position.y

    print("Moving to goal point " + str([goal_x, goal_y]) + " by potential fields")

    loop = rospy.Rate(20)

    global laser_readings



    # TODO:

    # Mueva el robot hacia el punto objetivo usando campos potenciales.

    # Recuerde que el punto de meta es un minimo local en el campo potencial, por lo tanto,

    # se puede alcanzar mediante el algoritmo de descenso de gradiente.

    # La suma de las fuerzas de atraccion y rechazo es el gradiente del campo potencial,

    # entonces, puedes llegar al punto de destino con el siguiente pseudocodigo:

    #

    #Establecemos constantes

    # Establecer epsilon (0.5 es un buen comienzo)

    epsilon = 0.5

    # Establecer tolerancia (0.1 es un buen comienzo)

    tol = 0.1

    # Obtener la posicion del robot respecto al mapa llamando a 

    robot_x, robot_y, robot_a = get_robot_pose(listener)

    # Calcular la distancia al objetivo como 

    error_global = 100000000

    error_local = 1000000000

    

    # MIENTRAS

    while error_global > tol and not rospy.is_shutdown():

        

        print("dentro de while global")

        # Calcula la fuerza de atraccion Fa llamando a 

        [fax,fay]=attraction_force(robot_x,robot_y,goal_x, goal_y)

        # Calcule la fuerza de rechazo Fr llamando a 

        [frx,fry]=rejection_force (robot_x,robot_y,robot_a,laser_readings)

        # Calcular la fuerza resultante en X y Y

        Fx,Fy = fax + frx, fay+fry

        print("F=",Fx, Fy)

    

        # Obtener la posicion del robot respecto al mapa

        robot_x, robot_y, robot_a = get_robot_pose(listener)

        # Calcular el siguiente punto objetivo local 

        x_lg, y_lg = robot_x - epsilon*Fx, robot_y - epsilon*Fy

        print("posicion robot=", robot_x, robot_y)

        print("Meta local=",x_lg,y_lg)



        while error_local > tol and not rospy.is_shutdown():

            print("dentro de segundo while")

            # Actualice la posicion del robot llamando a 

            robot_x, robot_y, robot_a = get_robot_pose(listener)



            # Calcule las seniales de control y le pasamos coordenadas de meta local

            msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, x_lg, y_lg)

            # Envie las seniales de control a la base movil llamando a 

            print("msg_cmd_vel", msg_cmd_vel)

            pub_cmd_vel.publish(msg_cmd_vel)

            



            # Llame a draw_force_markers(robot_x, robot_y, afx, afy, rfx, rfy, fx, fy, pub_markers) para dibujar todas las fuerzas

            draw_force_markers(robot_x, robot_y, fax, fay, frx, fry, Fx, Fy, pub_markers)



            # Calcule el error local como la magnitud del vector desde la posicion 

            # del robot hasta el punto objetivo local

            error_local = math.sqrt((x_lg - robot_x)**2 + (y_lg - robot_y)**2)

            # Espera un poco de tiempo llamando a 

            loop.sleep()



        # Actualice la posicion del robot llamando a 

        robot_x, robot_y, robot_a = get_robot_pose(listener)

        # Recalcular la distancia a la posicion de destino

        error_global = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)

        error_local = math.sqrt((x_lg - robot_x)**2 + (y_lg - robot_y)**2)

    

    # Publique una velocidad cero (para detener el robot despues de alcanzar el punto objetivo)

    msg_cmd_vel.linear.x = 0

    msg_cmd_vel.angular.z = 0

    pub_cmd_vel.publish(msg_cmd_vel)

    print("Goal point reached")



def get_robot_pose(listener):

    try:

        # consultamos al listener para una transformacion especifica por lookupTransform

        # queremos la trasformacion desde el sistema 'base_link' a 'map'y queremos la ultima transformacion disponible 

        #

        #  Esta funcion devuelve dos listas. La primera es la transformacion lineal (x, y, z) 

        # del cuadro secundario en relacion con el padre, y la segunda es el cuaternion (x, y, z, w) 

        # necesario para rotar desde la orientacion principal a la orientacion secundaria.

        # Todo esto esto envuelto en un bloque try-except para detectar posibles excepciones.

        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))

        a = 2*math.atan2(rot[2], rot[3])

        a = a - 2*math.pi if a > math.pi else a

        return [x, y, a]

    except:

        pass

    return [0,0,0]



def callback_scan(msg):

    global laser_readings

    laser_readings = [[msg.ranges[i], msg.angle_min+i*msg.angle_increment] for i in range(len(msg.ranges))]



def draw_force_markers(robot_x, robot_y, attr_x, attr_y, rej_x, rej_y, res_x, res_y, pub_markers):

    pub_markers.publish(get_force_marker(robot_x, robot_y, attr_x, attr_y, [0,0,1,1]  , 0))

    pub_markers.publish(get_force_marker(robot_x, robot_y, rej_x,  rej_y,  [1,0,0,1]  , 1))

    pub_markers.publish(get_force_marker(robot_x, robot_y, res_x,  res_y,  [0,0.6,0,1], 2))



def get_force_marker(robot_x, robot_y, force_x, force_y, color, id):

    hdr = Header(frame_id="map", stamp=rospy.Time.now())

    mrk = Marker(header=hdr, ns="pot_fields", id=id, type=Marker.ARROW, action=Marker.ADD)

    mrk.pose.orientation.w = 1

    mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = color

    mrk.scale.x, mrk.scale.y, mrk.scale.z = [0.07, 0.1, 0.15]

    mrk.points.append(Point(x=robot_x, y=robot_y))

    mrk.points.append(Point(x=(robot_x - force_x), y=(robot_y - force_y)))

    return mrk



def main():

    global listener, pub_cmd_vel, pub_markers

    print ("PRACTICE 05 - " + NAME)

    rospy.init_node("practice05")

    # Suscriptor al topico /scan

    rospy.Subscriber("/scan", LaserScan, callback_scan)  

    # Suscriptor al topico /move_base_simple/goal 

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_pot_fields_goal)

    # Crea un objeto publicador que publica en /cmd_vel mensajes de tipo Twist

    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist,  queue_size=10)

    # Crea un objeto publicador que publica en /navigation/pot_field_markers

    pub_markers = rospy.Publisher('/navigation/pot_field_markers', Marker, queue_size=10)

    #tf.TransformListener facilita la tarea de recibir transformaciones.

    # Aqui, creamos un objeto tf.TransformListener. Una vez que se crea el listener, 

    # comienza a recibir transformaciones tf y las almacena en bufer por hasta 10 segundos.

    listener = tf.TransformListener()

    rospy.spin()



if __name__ == '__main__':

    try:

        main()

    except rospy.ROSInterruptException:

        pass

    
