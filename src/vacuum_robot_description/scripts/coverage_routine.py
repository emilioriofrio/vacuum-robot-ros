#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# LISTA DE PUNTOS (Tu configuración actual)
lista_de_limpieza = [
    [-2.57, 0.0717],    # P1 (Pasillo)
    [4.43, 0.00922],    # P2 (Pasillo)
    [4.36, 3.17],       # P3 (Dormitorio)
    [0.669, 3.14],      # P4 (Dormitorio)
    [0.423, 0.427],     # P5 (Dormitorio)
    [-0.535, 0.671],    # P6 (Dormitorio)
    [-0.691, 3.11],     # P7 (Comedor)
    [-1.97, 2.14],      # P8 (Comedor)
    [-4.21, 2.27],      # P9 (Comedor)
    [-4.3, 3.29],       # P10 (Comedor)
    
    # --- ZONA BLOQUEADA (Cuarto 1) ---
    # Pon obstáculos en la puerta en Gazebo.
    # El robot irá, verá la puerta cerrada, intentará recalcular y ABORTARÁ.
    [-4.17, -3.32],     # P11 (Inaccesible)
    [-0.536, -3.28],    # P12 (Inaccesible)
    
    [-0.397, -0.521],   # P13 (Accesible)
    [0.607, -0.572],    # P14 (Accesible)
    
    # --- ZONA LIBRE (Cuarto 2) ---
    [0.865, -3.08],     # P15 (Cuarto 2)
    [4.28, -3.04],      # P16 (Cuarto 2)
    [4.0, -0.102]       # P17 (Cuarto 2) 
]

def ir_al_punto(x, y):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo(f"--> Navegando a: X={x}, Y={y}")
    client.send_goal(goal)
    
    # --- CAMBIO PROFESIONAL ---
    # Ponemos un tiempo MUY largo solo para evitar cuelgues extremos,
    # pero confiamos en que move_base nos avise si falla antes.
    terminado = client.wait_for_result(rospy.Duration(300.0)) # 5 minutos de margen

    if not terminado:
        rospy.logwarn("ALERTA: El robot se ha quedado colgado. Cancelando por seguridad.")
        client.cancel_goal()
        return False

    # Analiza CÓMO terminó
    estado = client.get_state()
    
    if estado == GoalStatus.SUCCEEDED:
        return True # Llegó bien
    elif estado == GoalStatus.ABORTED:
        rospy.logerr("OBSTÁCULO DETECTADO: El robot no encuentra camino. Zona inalcanzable.")
        return False # Esto hará que el bucle salte al siguiente punto
    elif estado == GoalStatus.REJECTED:
        rospy.logerr("META RECHAZADA: Coordenada inválida.")
        return False
    else:
        rospy.logwarn(f"Meta no completada. Estado: {estado}")
        return False

if __name__ == '__main__':
    try:
        rospy.init_node('vacuum_coverage_node')
        rospy.loginfo("INICIANDO LIMPIEZA INTELIGENTE (Fault Tolerant)")
        
        for i, punto in enumerate(lista_de_limpieza):
            rospy.loginfo(f"Zona {i+1}/{len(lista_de_limpieza)}")
            
            # Intentar llegar
            exito = ir_al_punto(punto[0], punto[1])
            
            if exito:
                rospy.loginfo("Zona alcanzada. Aspirando...")
                rospy.sleep(2) 
            else:
                # AQUÍ ESTÁ LA MAGIA:
                # Si ir_al_punto devuelve False (porque abortó por obstáculo),
                # el código entra aquí inmediatamente y pasa al siguiente "for".
                rospy.logwarn(f"Saltando Zona {i+1} debido a bloqueo.")

        rospy.loginfo("LIMPIEZA FINALIZADA")
        ir_al_punto(0.0, 0.0)
        
    except rospy.ROSInterruptException:
        pass
