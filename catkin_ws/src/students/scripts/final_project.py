#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# FINAL PROJECT - SIMPLE SERVICE ROBOT
# 
# Instrucciones:
# Escriba el codigo necesario para que el robot realice los siguientes comandos posibles:
# * Robot take the <pringles|drink> to the <table|kitchen>
# Puede elegir donde se ubican la mesa y la cocina dentro del mapa.
# Pringles y Drink son los dos objetos sobre la mesa utilizados en la practica 07.
# El robot debe reconocer las ordenes mediante el reconocimiento de voz.
# No se permite ingresar el comando por texto o de manera similar.
# El Robot debe anunciar el progreso de la accion usando sintesis de voz,
# for example: I'm going to grab..., I'm going to navigate to ..., I arrived to..., etc.

# por ejemplo: voy a agarrar (I'm going to grab)..., voy a navegar a (I'm going to navigate to)..., 
# llegue a (I arrived to)..., etc.
# Publicadores y suscriptores para interactuar con los subsistemas (navegacion,
# vision, manipulacion, sintesis de voz y reconocimiento) ya estan declarados.
#

from ast import And
from imp import is_frozen
from turtle import st
import rospy
import tf
import math
import time
import numpy
from std_msgs.msg import String, Float32MultiArray, Float32, Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, PointStamped
from sound_play.msg import SoundRequest
from custom_msgs.srv import *

NAME = "GONZALEZ_JIMENEZ_ITZEL"

listener_base    = None
#
# Global variable 'speech_recognized' contains the last recognized sentence
#
def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    # Si se esta ejecuntando una tarea deja de almacenar comandos de voz entrantes y de imprimir en pantalla
    if executing_task:  
        return
    new_task = True
    recognized_speech = msg.data
    print(recognized_speech)

#
# La variable global 'goal_reached' se establece en True cuando se alcanza el ultimo objetivo
#  de navegacion enviado
def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data


def parse_command(cmd):
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    loc = [8.0,8.5] if "TABLE" in cmd else [3.22, 9.72]  # Ubicacion de table y kitchen
    #return obj, loc
    return loc

#
# Esta funcion publica una posicion de objetivo global. Este topico esta suscrito a por
# pratice04 y realiza la planificacion y el seguimiento de rutas.
#
def go_to_goal_pose(goal_x, goal_y):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    print("antes de publicar en goa pose")
    pubGoalPose.publish(goal_pose)
    print("despues de publicar en goal pose")

#
# Esta funcion envia la posicion articular de meta al brazo izquierdo y espera 2 segundos
# para permitir que el brazo alcance la posicion deseada. 
#
def move_left_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubLaGoalPose
    msg = Float32MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubLaGoalPose.publish(msg)
    time.sleep(2.0)

#
# Esta funcion envia la posicion angular del objetivo a la pinza izquierda y espera 1 segundo
# para permitir que la pinza alcance el angulo objetivo. 
#
def move_left_gripper(q):
    global pubLaGoalGrip
    pubLaGoalGrip.publish(q)
    time.sleep(1.0)


def ik_left_arm(X,Y,Z,ROLL,PITCH,YAW):
    msg_la_ik = InverseKinematicsRequest()
    msg_la_ik.x = X
    msg_la_ik.y = Y
    msg_la_ik.z = Z
    msg_la_ik.roll = ROLL 
    msg_la_ik.pitch = PITCH
    msg_la_ik.yaw = YAW

    result_la_ik = clt_la_inverse_kin(msg_la_ik)
    return result_la_ik


#
# Esta funcion envia la posicion articular de meta al brazo derecho y espera 2 segundos
# para permitir que el brazo alcance la posicion deseada. 
#
def move_right_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubRaGoalPose
    msg = Float32MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubRaGoalPose.publish(msg)
    time.sleep(2.0)

#
# Esta funcion envia la posicion angular objetivo a la pinza derecha y espera 1 segundo
# para permitir que la pinza alcance el angulo objetivo.
#
def move_right_gripper(q):
    global pubRaGoalGrip
    pubRaGoalGrip.publish(q)
    time.sleep(1.0)


def ik_right_arm(X,Y,Z,ROLL,PITCH,YAW):
    msg_ra_ik = InverseKinematicsRequest()
    msg_ra_ik.x = X
    msg_ra_ik.y = Y
    msg_ra_ik.z = Z
    msg_ra_ik.roll = ROLL 
    msg_ra_ik.pitch = PITCH
    msg_ra_ik.yaw = YAW

    result_ra_ik = clt_ra_inverse_kin(msg_ra_ik)  
    return result_ra_ik

#
# Esta funcion envia los angulos de giro e inclinacion del objetivo a la cabeza y espera 1 segundo
# para permitir que la cabeza alcance la posicion de destino. 
#
def move_head(pan, tilt):
    global pubHdGoalPose
    msg = Float32MultiArray()
    msg.data.append(pan)
    msg.data.append(tilt)
    pubHdGoalPose.publish(msg)
    time.sleep(1.0)

#
# Esta funcion envia una velocidad lineal y angular a la base movil para realizar
# movimientos de bajo nivel. La base movil se movera a las velocidades angulares lineales dadas
# durante un tiempo dado por 't'
#
def move_base(linear, angular, t):
    global pubCmdVel
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pubCmdVel.publish(cmd)
    time.sleep(t)
    pubCmdVel.publish(Twist())


#
# Esta funcion envia un texto a sintetizar.
#
def say(text):
    global pubSay
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 1.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pubSay.publish(msg)
    time.sleep(1.5)


def find_object(obj):

    vision_msg = FindObjectRequest()
    vision_msg.cloud = rospy.wait_for_message("/kinect/points",PointCloud2)
    vision_msg.name = obj
    result = clt_find_object(vision_msg)
    coord = [result.x, result.y, result.z]
    return coord

def kinect_to_shoulder(coord):
    # Una vez que se crea el oyente, comienza a recibir transformaciones tf a traves del cable y 
    # las almacena en bufer por hasta 10 segundos.
    listener = tf.TransformListener()
    p_k = PointStamped()# Creamos el mensaje para el metodo transformPoint
    p_k.header.frame_id = 'kinect_link'   # Sistema coordenado de los puntos 
    p_k.header.stamp = rospy.Time(0)  # Queremos la ultima transformacion
    p_k.point.x, p_k.point.y, p_k.point.z = coord[0], coord[1], coord[2]
    target_frame = "shoulders_left_link"
    p_sh_la = listener.transformPoint(target_frame, p_k)

    return p_sh_la

# Para conocer la pose del robot
def get_robot_pose():
    try:
        ([x, y, z], rot) = listener_base.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]


def main():
    # Define variables globales 
    global new_task, recognized_speech, executing_task, goal_reached
    global pubLaGoalPose, pubRaGoalPose, pubHdGoalPose, pubLaGoalGrip, pubRaGoalGrip
    global pubGoalPose, pubCmdVel, pubSay
    global pub_point, img_bgr
    global listener_base
    print("FINAL PROJECT - " + NAME)
    rospy.init_node("final_exercise")
    # Suscriptor al topico reconociomiento de voz
    rospy.Subscriber('/recognized', String, callback_recognized_speech)
    # Suscriptor al topico que communica si se alcanzo la posicion deseada
    rospy.Subscriber('/navigation/goal_reached', Bool, callback_goal_reached)
    listener_base = tf.TransformListener()  # Crea un objeto de la clase tf     

    pubGoalPose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pubCmdVel   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubSay      = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    pubLaGoalPose = rospy.Publisher("/hardware/left_arm/goal_pose" , Float32MultiArray, queue_size=10);
    pubRaGoalPose = rospy.Publisher("/hardware/right_arm/goal_pose", Float32MultiArray, queue_size=10);
    pubHdGoalPose = rospy.Publisher("/hardware/head/goal_pose"     , Float32MultiArray, queue_size=10);
    pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper" , Float32, queue_size=10);
    pubRaGoalGrip = rospy.Publisher("/hardware/right_arm/goal_gripper", Float32, queue_size=10);
    pub_point = rospy.Publisher('/detected_object', PointStamped, queue_size=10) 

    loop = rospy.Rate(10)

    print("Waiting for services...")
    rospy.wait_for_service('/vision/find_object')
    rospy.wait_for_service('/manipulation/la_inverse_kinematics')
    rospy.wait_for_service('/manipulation/ra_inverse_kinematics')
    print("Services are now available.")
    global clt_la_inverse_kin
    global clt_ra_inverse_kin
    global clt_find_object
    clt_la_inverse_kin = rospy.ServiceProxy("/manipulation/la_inverse_kinematics", InverseKinematics)
    clt_ra_inverse_kin = rospy.ServiceProxy("/manipulation/ra_inverse_kinematics", InverseKinematics)
    clt_find_object = rospy.ServiceProxy("/vision/find_object", FindObject)

    new_task = False
    executing_task = False
    recognized_speech = ""
    goal_reached = False

    current_state = "STATE0"#"SM_INIT"
    requested_object   = ""
    requested_location = [0,0]

    #
    # FINAL PROJECT
    # AQUI SE COLOCA LA MAQUINA DE ESTADOS
    # 

    while not rospy.is_shutdown():

        if current_state == "STATE0":     # Estado de inicio
            say("Hello, I am waiting for new task")
            print("Iniciando maquina de estados...")
            time.sleep(2)
            current_state = "STATE1"
        
       
        elif current_state == "STATE1":   # Esperando comandos de voz***********************
            print("E1:Esperando comandos de voz")
            time.sleep(1)
            requestt = recognized_speech
            time.sleep(2)
            if "ROBOT" in requestt:
                print("Se ha hecho una peticion")
                say("has been requested"+recognized_speech)
                time.sleep(3)
                #parse_command(recognized_speech)
                current_state = "STATE2"
                
            elif "STOP" in requestt:
                say("executing STOP")
                move_base(0, 0, 1)
                time.sleep(1)
                current_state = "STATE1"

            else:                
                print("No se han recibido nuevas peticiones")
                print("Regresando a E1...")
                current_state = "STATE1"

            
        elif current_state == "STATE2":     # MOVE HEAD
            print("EDO2: Baja la cabeza")
            say("Executing object recognition")
            time.sleep(2)
            move_head(0, -0.9)
            current_state = "STATE3"

        elif current_state == "STATE3":  # OBJECT RECONGNICION
            print("imprime",recognized_speech)
            obj_target = recognized_speech
            loc_target = recognized_speech
            listener = tf.TransformListener()
            p_k = PointStamped()# Creamos el mensaje para el metodo transformPoint
            p_k.header.frame_id = 'kinect_link'   # Sistema coordenado de los puntos 
            p_k.header.stamp = rospy.Time(0)  # Queremos la ultima transformacion
            obj = ""
            if "PRINGLES" in obj_target:
                if "KITCHEN" in loc_target:
                    print("Se pidio llevar PRINGLES a la cocina")

                print("Se pidio llevar PRINGLES a la mesa")
                obj = "pringles"
                c_pringles = find_object(obj)
                time.sleep(3)
                #print("coordenadas pringles en frame Kinect: ***",c_pringles)            
                p_k.point.x, p_k.point.y, p_k.point.z = c_pringles[0], c_pringles[1], c_pringles[2]
                target_frame = "shoulders_left_link"
                c_pringles = listener.transformPoint(target_frame, p_k) # Making change of frames
                #print("coordenadas  en frame Shoulder_l****: \n",c_pringles.point)
            
            else:
                if "KITCHEN" in loc_target:
                    print("Se pidio llevar DRINK a la cocina")

                print("Se pidio llevar DRINK a la mesa")
                obj = "Drink"
                c_drink = find_object(obj)
                time.sleep(3)
                #print("coordenadas Drink en frame kinect: ****",c_drink)
                p_k.point.x, p_k.point.y, p_k.point.z = c_drink[0], c_drink[1], c_drink[2]
                target_frame = "shoulders_right_link"
                c_drink = listener.transformPoint(target_frame, p_k)    # Making change of frames
                #print("coordenadas Drink en frame Shoulder_r: ***",c_drink.point)
            
            current_state = "STATE4"
        
        elif current_state == "STATE4":  # PREPARING TO TAKE AN OBJECT
            print("Prepare")
            say("Executing PREPARING TO TAKE AN OBJECT")
            move_base(-0.2, 0, 0.3)
            time.sleep(1)

            if obj == "pringles":
                move_left_arm(-0.7,0,0,1,0,1,0)
                move_left_gripper(0.5)
            else:
                move_right_arm(-0.7,0,0,1,0,1,0)
                move_right_gripper(0.5)
            
            current_state = "STATE5"


        elif current_state == "STATE5": # CINEMATICA INVERSA
            current_state = "STATE6"
            say("Executing INVERSE KINEMATIC")
            time.sleep(2)
            print("Calculando cinematica inversa de punto centroide...")
            if obj == "pringles":
                ik_la_resp = ik_left_arm(c_pringles.point.x, c_pringles.point.y, c_pringles.point.z,0,-1.7,0)
                #print("IK DE BRAZO IZQ*****",ik_la_resp)
                
            else:
                ik_ra_resp = ik_right_arm(c_drink.point.x, c_drink.point.y, c_drink.point.z+0.06,0,-1.7,0)
                #print("IK DE BRAZO DER*****",ik_ra_resp)
                
            current_state == "STATE6"


        elif current_state == "STATE6": # MOVE ARM
            current_state = "STATE7"
            say("Executing move arm ")
            time.sleep(1)
            if obj == "pringles":
                move_left_arm(ik_la_resp.q1 ,ik_la_resp.q2, ik_la_resp.q3, ik_la_resp.q4, ik_la_resp.q5,
                            ik_la_resp.q6, ik_la_resp.q7)
                move_base(0.1, 0, 1.3)
                move_left_gripper(-0.2)
                time.sleep(0.7)
                move_left_arm(ik_la_resp.q1, ik_la_resp.q2, ik_la_resp.q3, ik_la_resp.q4+0.170, ik_la_resp.q5,
                            ik_la_resp.q6, ik_la_resp.q7)
                time.sleep(1)
            

            else:
                move_right_arm(ik_ra_resp.q1, ik_ra_resp.q2, ik_ra_resp.q3, ik_ra_resp.q4, ik_ra_resp.q5,
                            ik_ra_resp.q6, ik_ra_resp.q7)
                move_base(0.1, 0, 1.2)
                move_right_gripper(-0.23)
                move_right_arm(ik_ra_resp.q1, ik_ra_resp.q2, ik_ra_resp.q3, ik_ra_resp.q4+0.170, ik_ra_resp.q5,
                            ik_ra_resp.q6, ik_ra_resp.q7)
                         


        elif current_state == "STATE7":  # GO TO GOAL LOCATION
            current_state = "STATE8"
            print("going to the target place....")
            say("Executing go to goal pose")
            print("goal reached:*****",goal_reached)
            move_base(-0.5, 0, 0.6)
            goal_x, goal_y = parse_command(loc_target)
            go_to_goal_pose(goal_x, goal_y)
            goal_reached = False
            while not goal_reached and (not rospy.is_shutdown()):
                time.sleep(1)
                print(" goal reached:*****",goal_reached)
            
            move_base(0, 0, 1)    # Se detiene por completo
            say("arrived at the required location")
            xx, yy, a = get_robot_pose()
            print("Se llego al lugar solicitado con Pose: ", xx,yy,a )
            time.sleep(2)
            
        elif current_state == "STATE8":  # ENTREGA OBJETO
            current_state = "STATE9"
            print("facing the table")
            say("Executing facing the table")
            time.sleep(2)
            move_base(0, 0, 1)    # Se detiene por completo
            time.sleep(3)
            if "TABLE" in loc_target:
                xx, yy, a = get_robot_pose()
                while a < -6.12 or a > -6.0:
                    move_base(0, -0.9, 0.1)    # Se orienta frente a la mesa
                    xx, yy, a = get_robot_pose()
                    print("Pose: ", xx,yy,a )
                    
                move_base(0, 0, 1)
                move_base(0.8, 0, 95)    # Hacia adelante

            else:
                xx, yy, a = get_robot_pose()
                while numpy.abs(a) > 1.55:    # a -> -1.57
                    move_base(0, 0.9, 0.1)    # Se orienta frente a la mesa
                    xx, yy, a = get_robot_pose()
                    #a = # Valor absoluto
                    print("Pose: ", xx,yy,a )
                    
                move_base(0, 0, 1)
                time.sleep(0.5)
                move_base(0.6, 0, 0.5)  # Hacia adelante
                print("Pose: ", xx,yy,a )
                time.sleep(3)

            move_right_gripper(0.3)    # Abre gripper
            move_left_gripper(0.3)    # Abre gripper
            move_base(-0.8, 0, 1.3)
            time.sleep(2)

            if obj == "pringles":   # REGRESA MANIPULADOR A POSICION HOME***********
                move_left_gripper(0)
                move_left_arm(0,0,0,0,0,0,0)
            else:
                move_right_gripper(0)
                move_right_arm(0,0,0,0,0,0,0)
        

        elif current_state == "STATE9":     # GO TO INITIAL POSITION**************
            print("Go to initial position")
            say("Executing Go to initial position")
            time.sleep(2)
            goal_x, goal_y = 3.25, 5.63
            print("Valor de goal_reached antes de regresar", goal_reached)
            go_to_goal_pose(goal_x, goal_y)
            goal_reached = False
            while not goal_reached and (not rospy.is_shutdown()):
                print("Dentro de segundo while")
                time.sleep(2)
                print(" goal reached:*****",goal_reached)
            
            move_base(0, 0, 2)    # Se detiene por completo
            time.sleep(3)

            xx, yy, a = get_robot_pose()
            while numpy.abs(a) > 1.59:    # a -> -1.57
                move_base(0, 0.9, 0.1)    # Se orienta frente a la mesa
                xx, yy, a = get_robot_pose()
                print("Pose: ", xx,yy,a )

            move_base(0, 0, 2)    # Se detiene por completo
            time.sleep(1)
            move_base(0.1, 0, 0.4)    # Se detiene por completo
            time.sleep(1)
            say("arrived at the initial location")
            xx, yy, a = get_robot_pose()
            print("Pose de llegada: ", xx,yy,a )
            time.sleep(5)
            
            current_state = "STATE1"
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
