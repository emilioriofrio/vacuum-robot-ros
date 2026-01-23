#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class VacuumSupervisor:
    def __init__(self):
        rospy.init_node('coverage_supervisor')
        
        # Conecta con move_base (el nodo que configuraste en navigation.launch)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Esperando al sistema de navegación...")
        self.client.wait_for_server()
        rospy.loginfo("¡Conectado! Iniciando rutina de limpieza.")

    def ir_a_punto(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map" # Navegamos respecto al mapa guardado
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Coordenadas destino
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0 # Mirando al frente

        rospy.loginfo(f"Aspirando hacia: x={x}, y={y}")
        self.client.send_goal(goal)
        
        # Esperar hasta llegar (60 segundos máximo)
        terminado = self.client.wait_for_result(rospy.Duration(60))
        
        if not terminado:
            rospy.logwarn("Obstáculo detectado o tiempo agotado. Saltando punto.")
            self.client.cancel_goal()
        else:
            rospy.loginfo("Zona limpia.")

    def iniciar_rutina(self):
        # --- PUNTOS DE LIMPIEZA ---
        # NOTA: Estas coordenadas (x, y) son metros desde el centro del mapa.
        # Puedes ver las coordenadas exactas poniendo el mouse en RViz.
        puntos = [
            (1.0, 0.0),   # Punto 1
            (1.5, 1.5),   # Punto 2
            (0.0, 1.5),   # Punto 3
            (-1.0, 0.0),  # Punto 4
            (0.0, 0.0)    # Volver a la base
        ]

        for p in puntos:
            self.ir_a_punto(p[0], p[1])
            rospy.sleep(1.0) # Simula tiempo aspirando

        rospy.loginfo("Rutina de limpieza terminada.")

if __name__ == '__main__':
    try:
        robot = VacuumSupervisor()
        robot.iniciar_rutina()
    except rospy.ROSInterruptException:
        pass
