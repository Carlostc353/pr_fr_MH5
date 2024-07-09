#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from markers import *
from proy_functions import *
from roslib import packages

import rbdl

if __name__ == '__main__':
    rospy.init_node("control_dininv")
    pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
    bmarker_actual  = BallMarker(color['RED'])
    bmarker_deseado = BallMarker(color['GREEN'])
    # Archivos donde se almacenara los datos
    fqact = open("/tmp/qactual.txt", "w")
    fqdes = open("/tmp/qdeseado.txt", "w")
    fxact = open("/tmp/xcurrent.txt", "w")
    fxdes = open("/tmp/xdesired.txt", "w")

    # Nombres de las articulaciones
    jnames = ['Revolute 1', 'Revolute 2', 'Revolute 3','Slider 4', 'Revolute 5', 'Revolute 6','Revolute 7']
    # Objeto (mensaje) de tipo JointState
    jstate = JointState()
    # Valores del mensaje
    jstate.header.stamp = rospy.Time.now()
    jstate.name = jnames

    # =============================================================
    # Configuracion articular inicial (en radianes)
    q = np.array([0.2, 0.2, 0.2, 0.025, 0.2, 0.2,0.2])
    # Velocidad inicial
    dq = np.array([0.1, 0.1, 0.1, 0.002, 0.1, 0.1, 0.1])
    # Aceleracion inicial
    ddq = np.array([0., 0., 0., 0., 0., 0.])
    # Configuracion articular deseada
    qdes = np.array([0, 0, 0, 0.025, 0, 0,0])
    # Velocidad articular deseada
    dqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # Aceleracion articular deseada
    ddqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # =============================================================


    # Posicion resultante de la configuracion articular deseada
    xdes = fkine_mh5lsii(qdes)[0:3,3]
    dxdes = np.array([0,0,0])
    ddxdes = np.array([0,0,0])

    # Copiar la configuracion articular en el mensaje a ser publicado
    jstate.position = q

    pub.publish(jstate)

    # Modelo RBDL
    modelo = rbdl.loadModel('/home/ctorres/lab_ws/src/mh5lsii_01_description/urdf/mh5lsii_01.urdf')
    ndof   = modelo.q_size     # Grados de libertad
    zeros = np.zeros(ndof)     # Vector de ceros

    # Frecuencia del envio (en Hz)
    freq = 20
    dt = 1.0/freq
    rate = rospy.Rate(freq)

    # Simulador dinamico del robot
    robot = Robot(q, dq, ndof, dt)

    # Bucle de ejecucion continua
    t = 0.0

    # Se definen las ganancias del controlador
    valores = 0.0000000000000000000000000000000000001*np.array([0.00001, 0.00001,0.00001, 0.00001, 0.00001, 0.00001,0.00001])
    Kp = np.diag(valores)
    Kd = 2*np.sqrt(Kp)
    J_PAS = np.zeros([3,ndof])
    epas = np.array([0,0,0])


    while not rospy.is_shutdown():

        # Leer valores del simulador
        q  = robot.read_joint_positions()
        dq = robot.read_joint_velocities()
        # Posicion actual del efector final
        x = fkine_mh5lsii(q)[0:3,3]
        # Tiempo actual (necesario como indicador para ROS)
        jstate.header.stamp = rospy.Time.now()

        
        # ----------------------------
        # Control dinamico (COMPLETAR)
        # ----------------------------
        
        b2 = np.zeros(ndof)             # Para efectos no lineales
        M2 = np.zeros([ndof, ndof])     # Para matriz de inercia
            
        # Matriz de Inercia
        rbdl.CompositeRigidBodyAlgorithm(modelo, q, M2)
        
        # Vector de efectos no lineales
        rbdl.NonlinearEffects(modelo, q, dq, b2)
        
        # Calculo de Jacobiano y su derivada
        Ja = jacobian_mh5lsii(q)
        dJa = (Ja - J_PAS) / dt
        
        # Calculo de error y derivada de error
        e = xdes - x
        de = (e-epas)/dt
        print('error')
        print(de)

        # Ley de control
        u = M2.dot(np.linalg.pinv(Ja)).dot(ddxdes - dJa.dot(dq) + Kd.dot(de) + Kp.dot(e)) + b2
        epas = copy(e)
        J_PAS = copy(Ja)

        # Simulacion del robot
        robot.send_command(u)

        # Almacenamiento de datos
        fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
        fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
        fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
        fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+'\n ')

        # Publicacion del mensaje
        jstate.position = np.concatenate((q, np.zeros(2)))
        pub.publish(jstate)
        bmarker_deseado.xyz(xdes)
        bmarker_actual.xyz(x)
        t = t+dt
        # Esperar hasta la siguiente  iteracion
        rate.sleep()

    fqact.close()
    fqdes.close()
    fxact.close()
    fxdes.close()
