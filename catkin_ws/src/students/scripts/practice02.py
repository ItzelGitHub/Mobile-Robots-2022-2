#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# PRACTICE 2 - PATH PLANNING BY A-STAR
#
# Instructions:
# Write the code necessary to plan a path using an
# occupancy grid and the A* algorithm
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

from cmath import cos
import numpy
import heapq
import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "GONZALEZ_JIMENEZ"

msg_path = Path()

def a_star(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    #
    # TODO:
    # Escribe el algoritmo A* para encontrar una ruta en un mapa de ocupacion dada la celda de inicio
    # [star_r, star_c], la celda meta [goal_r, goal_c] y el mapa 'grid map'
    # Devuelve un conjunto de puntos de la forma [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # indicando los indices (coordenadas de celda) de las celdas de ruta.
    # Si no se pudo encontrar la ruta, devuelve una tupla[] vacia
    #

    n_start = [start_r, start_c] # Es 225,363
    n_goal = [goal_r, goal_c]
    print("NODO DE INICIO: **************", n_start)
    print("NODO DE meta: **************", n_goal)
    adjacents = []  # Vecinos
    dist = 1    # Distancia de nodo actual a vecinos
    diagona_dist = dist*1.4142
    open_list = []   # Lista abierta que guarda nodos por visitar y su costo de paso
    # Lista cerrada que contiene nodos visitados 
    closed_list = []#[[226,363], [224,363]]
    parents = {}# Registro de nodos hijos y padres
    g_costs = numpy.full(grid_map.shape, float("inf"))
    f_costs = numpy.full(grid_map.shape, float("inf"))   
    # En la coordenada del nodo de inicio establecemos costo cero para g y f
    g_costs[start_r, start_c] = 0
    f_costs[start_r, start_c] = 0
    open_list.append([[start_r, start_c], 0])    # Agrega nodo de inicio a lista abierta
    path_found = False    
    n_start = [start_r, start_c] # Es 225,363
    n_goal = [goal_r, goal_c]
    print("NODO DE INICIO: **************", n_start)
    print("NODO DE meta: **************", n_goal)
    adjacents = []# Vecinos
    dist = 1    # Distancia de nodo actual a vecinos
    diagona_dist = dist*1.4142
    open_list = []  # Lista abierta que guarda nodos por visitar y su costo de paso
    # Lista cerrada que contiene nodos visitados 
    closed_list = []#[[226,363], [224,363]]
    parents = {}    # Registro de nodos hijos y padres
    g_costs = numpy.full(grid_map.shape, float("inf"))
    f_costs = numpy.full(grid_map.shape, float("inf"))   
    # En la coordenada del nodo de inicio establecemos costo cero para g y f
    g_costs[start_r, start_c] = 0
    f_costs[start_r, start_c] = 0
    open_list.append([[start_r, start_c], 0])    # Agrega nodo de inicio a lista abierta
    path_found = False
    path = []   # Guarda la trayectoria mas corta
    rospy.loginfo("Inicializacion finalizada")
    while open_list:
        # Verifica que el punto deseado este dentro de los limites del mapa
        if goal_r > 512 or goal_c > 892 or  goal_r < 0 or  goal_c < 0:
            print("Objetivo fuera de mapa")
            break
        # Verifica que el punto deseado no este dentro de un obstaculo
        if grid_map[goal_r, goal_c] != 0:
            print("El objetivo esta sobre un obstaculo, intente con otro punto")
            break

        if grid_map[start_r, start_c] != 0:
            print("El se encuentra sobre un obstaculo, modifique su posicion")
            break
        
        # Ordena open_list de acuerdo con el costo f mas bajo [[[x1,y1],f1], [[x2,y2],f2],...]
        open_list.sort(key = lambda x: x[1])
        # Extrae el primer elemento (coordenada)
        n_actual = open_list.pop(0)[0]
        # Nodo Actual***************************
        r_actual = n_actual[0]
        c_actual = n_actual[1]

        # Agrega nodo actual a lista cerrada
  #      if not n_actual in closed_list:
        closed_list.append(n_actual)
        # Si nodo actual es nodo meta salimos del ciclo
        if n_actual == n_goal:
            path_found = True
            break
        
        # Vecinos de nodo actual
        # Superior
        if grid_map[r_actual-1, c_actual] <= 50:     # verifica si la celda no esta ocupada
             # suma la distancia al vecino con costo en ese nodo
            cost = dist + cost_map[r_actual-1, c_actual]   
            adjacents.append([[r_actual-1, c_actual], cost])  # agrega elemento a lista vecinos
            #print("registra nodo arriba, costo = ", cost)
        # Izqierda
        if grid_map[r_actual, c_actual-1] <= 50:
            cost = dist + cost_map[r_actual, c_actual-1]
            adjacents.append([[r_actual, c_actual-1], cost])
            #print("registra nodo izq, costo = ", cost)
        # Derecha
        if grid_map[r_actual, c_actual+1] <= 50:
            cost = dist + cost_map[r_actual, c_actual+1]
            adjacents.append([[r_actual, c_actual+1], cost])
            #print("registra nodo der, costo = ", cost)
        # Inferior
        if grid_map[r_actual+1, c_actual] <= 50:
            cost = dist + cost_map[r_actual+1, c_actual]
            adjacents.append([[r_actual+1, c_actual], cost])
            #print("registra nodo abajo, costo = ", cost)
        j = 0
        # Analiza los nodos vecinos*************************************************
        for adjacent, cost in adjacents:
            adj_r = adjacent[0]
            adj_c = adjacent[1]

            if adjacent in closed_list:     # si ya esta en lista cerrada lo omite
                #print("Elemento repetido en vecinos: ",adjacent)
                continue
        # Calcula g_cost
        # g = g del padre + 1 si el paso es ortogonal o 1.4142 si el paso es diagonal
        # mas el costo en esa celda
            g_cost = g_costs[r_actual, c_actual] + cost  # Costo de paso + costo heredado
        # Actualiza el g_gost de la celda del vecino en mapa g_costs
            g_costs[adj_r, adj_c] = g_cost
            h =  abs(goal_c-adj_r)+abs(goal_c-adj_c)# heuristica , distancia de manhattan
            f_cost = g_cost + h
  
            in_open_list = False
            index = 0
      
            for i in range(len(open_list)):
                #print("i de 2o FOR: ", i)
                # Verifica si nodo vecino esta en lista abierta
                if open_list[i][0] == adjacent:
                 #   print("nodo vecino ya esta en la lista")
                    in_open_list = True
                    index = i
                    break

            if in_open_list:
                if f_cost < f_costs[adj_r, adj_c]:  # Si costo actual es menor que costo anterior de nodo vecino
                # actualiza el valor de f_cost
                    f_costs[adj_r, adj_c] = f_cost
                # actualiza la lista de parientes
                    parents[adj_r, adj_c] = n_actual
                # actualiza lista abieta
                    open_list[i] = [adjacent, f_cost]
                    #print("\nse actualizo costo f"
            # Caso 2: nodo vecino no esta en la lista abierta
            else:
                # Actualiza f_cost
                f_costs[adj_r, adj_c] = f_cost

                parents[adj_r, adj_c] = n_actual

                open_list.append([adjacent, f_cost])
            j += 1
        #IMPLEMENTAR ALGORITMO PARA VACIAR LISTA DE VECINOS*******************************************
        adjacents = []

    if not path_found:
        rospy.logwarn('No se encontro una ruta')
        return path

    if path_found:
        # Nodo debe ser una tupla
        nodo = goal_r, goal_c
        inicio = start_r, start_c
        print("LLEGAMOS A LA PARTE DE ORDENAMIENTO DE RUTA**********************************************")
        path.append(nodo)
        while nodo != inicio:
        # obtiene el siguiente nodo PERO ESTAMOS SACANDO UNA LISTA! ES NECESARIO CONVERTIRLA A TUPLA!!!************
            nodo_list = parents[nodo]
            # Conversion a tupla
            nodo = nodo_list[0], nodo_list[1]
            path.append(nodo)

        # Reconstruccion de la trayectoria
        path = path[::-1]
        print("si se encontro ruta")
        
    return path

def get_maps():
    print("Getting inflated and cost maps...")
    clt_static_map = rospy.ServiceProxy("/static_map"  , GetMap)
    clt_cost_map   = rospy.ServiceProxy("/cost_map"    , GetMap)
    clt_inflated   = rospy.ServiceProxy("/inflated_map", GetMap)
    try:
        static_map = clt_static_map().map
    except:
        print("Cannot get static map. Terminating program. ")
        exit()
    try:
        inflated_map = clt_inflated().map
        cost_map     = clt_cost_map().map
        print("Using inflated map with " +str(len(inflated_map.data)) + " cells.")
        print("Using cost map with "     +str(len(cost_map.data))     + " cells.")
    except:
        inflated_map = static_map
        cost_map     = static_map
        print("Cannot get augmented maps. Using static map instead.")
    inflated_map = numpy.reshape(numpy.asarray(inflated_map.data), (static_map.info.height, static_map.info.width))
    cost_map     = numpy.reshape(numpy.asarray(cost_map.data)    , (static_map.info.height, static_map.info.width))
    return [static_map, inflated_map, cost_map]

def callback_a_star(req):
    [s_map, inflated_map, cost_map] = get_maps()
    res = s_map.info.resolution
    [sx, sy] = [req.start.pose.position.x, req.start.pose.position.y]
    [gx, gy] = [req.goal .pose.position.x, req.goal .pose.position.y]
    [zx, zy] = [s_map.info.origin.position.x, s_map.info.origin.position.y]
    print("Calculating path by A* from " + str([sx, sy])+" to "+str([gx, gy]))
    path = a_star(int((sy-zy)/res), int((sx-zx)/res), int((gy-zy)/res), int((gx-zx)/res), inflated_map, cost_map)

    msg_path.poses = []
    for [r,c] in path:
        msg_path.poses.append(PoseStamped(pose=Pose(position=Point(x=(c*res + zx), y=(r*res + zy)))))
    return GetPlanResponse(msg_path)

def main():
    print("PRACTICE 02 - " + NAME)
    rospy.init_node("practice02")
    rospy.wait_for_service('/static_map')
    rospy.Service('/path_planning/a_star_search'  , GetPlan, callback_a_star)
    pub_path = rospy.Publisher('/path_planning/a_star_path', Path, queue_size=10)
    loop = rospy.Rate(2)
    msg_path.header.frame_id = "map"
    
    while not rospy.is_shutdown():
        pub_path.publish(msg_path)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
