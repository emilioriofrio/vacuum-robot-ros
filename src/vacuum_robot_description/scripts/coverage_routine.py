#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# CONFIGURACIÓN DE LA RUTA DE LIMPIEZA

# [Coordenada X, Coordenada Y]
lista_de_limpieza = [
    [-2.57, 0.0717],    # Punto 1 (Pasillo)
    [4.43, 0.00922],    # Punto 2 (Pasillo)
    [4.36, 3.17],       # Punto 3 (Dormitorio)
    [0.669, 3.14],      # Punto 4 (Dormitorio)
    [0.423, 0.427],     # Punto 5 (Dormitorio)
    [-0.535, 0.671],    # Punto 6 (Dormitorio)
    [-0.691, 3.11],     # Punto 7 (Comedor)
    [-1.97, 2.14],      # Punto 8 (Comedor)
    [-4.21, 2.27],      # Punto 9 (Comedor)
    [-4.3, 3.29],       # Punto 10 (Comedor)
    [-4.17, -3.32],     # Punto 11 (Cuarto 1)
    [-0.536, -3.28],    # Punto 12 (Cuarto 1)
    [-0.397, -0.521],   # Punto 13 (Cuarto 1)
    [0.607, -0.572],    # Punto 14 (Cuarto 1)
    [0.865, -3.08],     # Punto 15 (Cuarto 2)
    [4.28, -3.04],      # Punto 16 (Cuarto 2)
    [4.0, -0.102]       # Punto 17 (Cuarto 2) 
]

def ir_al_punto(x, y):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # Posición
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0 # Mirando al frente

    rospy.loginfo(f"--> Aspiradora yendo a: X={x}, Y={y}")
    client.send_goal(goal)
    
    # Esperar a que llegue
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Error: El servidor de navegación no responde.")
        return False
    return True

if __name__ == '__main__':
    try:
        rospy.init_node('vacuum_coverage_node')
        rospy.loginfo("INICIANDO PROTOCOLO DE LIMPIEZA AUTOMÁTICA")
        
        for i, punto in enumerate(lista_de_limpieza):
            rospy.loginfo(f"Limpiando zona {i+1} de {len(lista_de_limpieza)}...")
            resultado = ir_al_punto(punto[0], punto[1])
            
            if resultado:
                rospy.loginfo("¡Zona alcanzada! Aspirando durante 3 segundos...")
                rospy.sleep(3) # Simula tiempo de aspirado
            else:
                rospy.logwarn("No se pudo llegar a esta zona, saltando a la siguiente...")

        rospy.loginfo("LIMPIEZA COMPLETADA: Regresando a la base")
        #Volver al punto 0,0 (Home) al final
        ir_al_punto(0.0, 0.0)
        
    except rospy.ROSInterruptException:
        pass
