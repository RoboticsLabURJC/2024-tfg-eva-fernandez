import pybullet as p
import pybullet_data as pd
import time
import json
import os
import numpy as np
import matplotlib.pyplot as plt

file_name = input("Inserta el nombre de tu fichero .json (no olvides la extensión): ")

while not file_name.endswith(".json"):
    file_name = input("No olvides la extensión .json: ")
    
name = "/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/movements/"+file_name

# Vaciar el json (o crearlo) para que sean todo datos nuevos
with open(name, "w") as json_file:
    json.dump([], json_file, indent=4)

time_to_go = 0
used_joints = [1,2,13,14,15,16,17,18,26,27,28,29,30,31,39,40,41,42,43,56,57,58,59,60]

# Preparar mundo y modelo
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(1.0, 89.60, -4.40, [0, 0, 0.5]) # Poner la cámara en la posición deseada
euler_angles = [0,0,0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0,0,0.35]
model = p.loadURDF("/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/Modelo_NAO/nao.urdf", startPosition, startOrientation)

# Sliders 
# Slider del tiempo y un slider para indicar que esa es la secuencia que quieres guardar
button = p.addUserDebugParameter("DONE", 0, 1, 0)
button_prev_value = p.readUserDebugParameter(button)

desired_time = p.addUserDebugParameter("Time", 0, 30, 0)

# Preparar sliders para movimiento
head_yaw = p.addUserDebugParameter("HeadYaw", -2.08567, 2.08567, 0)
head_pitch = p.addUserDebugParameter("HeadPitch", -0.671952, 0.514872, 0)

# LADO IZQUIERO
L_hip_yaw_pitch = p.addUserDebugParameter("LHipYawPitch", -1.14529, 0.740718, 0)
L_hip_pitch = p.addUserDebugParameter("LHipPitch", -1.53589, 1.53589, -0.179)
L_hip_roll = p.addUserDebugParameter("LHipRoll", -0.379435, 0.79046, 0)

L_knee_pitch = p.addUserDebugParameter("LKneePitch", -0.0923279, 2.11255, 0.698132)

L_ankle_pitch = p.addUserDebugParameter("LAnklePitch", -1.18944, 0.922581, -0.479)
L_ankle_roll = p.addUserDebugParameter("LAnkleRoll", -0.397761, 0.768992, 0)

L_shoulder_pitch = p.addUserDebugParameter("LShoulderPitch", -5.08567, 5.08567, 1.39626)
L_shoulder_roll = p.addUserDebugParameter("LShoulderRoll", -0.314159, 1.32645, 0.198132)

L_elbow_yaw = p.addUserDebugParameter("LElbowYaw", -2.08567, 2.08567, -1.39626)
L_elbow_roll = p.addUserDebugParameter("LElbowRoll", -1.54462, -0.0349066, -1.0472)

L_wrist_yaw = p.addUserDebugParameter("LWristYaw", -1.82387, 1.82387, -0.192)

# LADO DERECHO
R_hip_yaw_pitch = p.addUserDebugParameter("RHipYawPitch", -1.14529, 0.740718, 0)
R_hip_pitch = p.addUserDebugParameter("RHipPitch", -1.53589, 1.53589, -0.179)
R_hip_roll = p.addUserDebugParameter("RHipRoll", -0.79046, 0.379435, 0)

R_knee_pitch = p.addUserDebugParameter("RKneePitch", -0.0923279, 2.11255, 0.698132)

R_ankle_pitch = p.addUserDebugParameter("RAnklePitch", -1.1863, 0.932006, -0.479)
R_ankle_roll = p.addUserDebugParameter("RAnkleRoll", -0.768992, 0.397935, 0)

R_shoulder_pitch = p.addUserDebugParameter("RShoulderPitch", -5.08567, 5.08567, 1.39626)
R_shoulder_roll = p.addUserDebugParameter("RShoulderRoll", -1.32645, 0.314159, -0.198132)

R_elbow_yaw = p.addUserDebugParameter("RElbowYaw", -2.08567, 2.08567, 1.39626)
R_elbow_roll = p.addUserDebugParameter("RElbowRoll", 0.0349066, 1.54462, 1.0472)

R_wrist_yaw = p.addUserDebugParameter("RWristYaw", -1.82387, 1.82387, 0.192)

# Preparar simulación
p.setRealTimeSimulation(True)
while True:
    # Controlar tiempo y si se quiere guardar el fotograma
    desired_time_value = int(round(p.readUserDebugParameter(desired_time)))
    decided = int(round(p.readUserDebugParameter(button)))

    # Controlar a NAO
    head_yaw_value = p.readUserDebugParameter(head_yaw)
    head_pitch_value = p.readUserDebugParameter(head_pitch)
    L_hip_yaw_pitch_value = p.readUserDebugParameter(L_hip_yaw_pitch)
    L_hip_pitch_value = p.readUserDebugParameter(L_hip_pitch)
    L_hip_roll_value = p.readUserDebugParameter(L_hip_roll)
    L_knee_pitch_value = p.readUserDebugParameter(L_knee_pitch)
    L_ankle_pitch_value = p.readUserDebugParameter(L_ankle_pitch)
    L_ankle_roll_value = p.readUserDebugParameter(L_ankle_roll)
    L_shoulder_pitch_value = p.readUserDebugParameter(L_shoulder_pitch)
    L_shoulder_roll_value = p.readUserDebugParameter(L_shoulder_roll)
    L_elbow_yaw_value = p.readUserDebugParameter(L_elbow_yaw)
    L_elbow_roll_value = p.readUserDebugParameter(L_elbow_roll)
    L_wrist_yaw_value = p.readUserDebugParameter(L_wrist_yaw)
    R_hip_yaw_pitch_value = p.readUserDebugParameter(R_hip_yaw_pitch)
    R_hip_pitch_value = p.readUserDebugParameter(R_hip_pitch)
    R_hip_roll_value = p.readUserDebugParameter(R_hip_roll)
    R_knee_pitch_value = p.readUserDebugParameter(R_knee_pitch)
    R_ankle_pitch_value = p.readUserDebugParameter(R_ankle_pitch)
    R_ankle_roll_value = p.readUserDebugParameter(R_ankle_roll)
    R_shoulder_pitch_value = p.readUserDebugParameter(R_shoulder_pitch)
    R_shoulder_roll_value = p.readUserDebugParameter(R_shoulder_roll)
    R_elbow_yaw_value = p.readUserDebugParameter(R_elbow_yaw)
    R_elbow_roll_value = p.readUserDebugParameter(R_elbow_roll)
    R_wrist_yaw_value = p.readUserDebugParameter(R_wrist_yaw)

    p.setJointMotorControl2(model,1, p.POSITION_CONTROL, targetPosition=head_yaw_value, maxVelocity=2)
    p.setJointMotorControl2(model,2, p.POSITION_CONTROL, targetPosition=head_pitch_value, maxVelocity=2)
    p.setJointMotorControl2(model,13, p.POSITION_CONTROL, targetPosition=L_hip_yaw_pitch_value, maxVelocity=2)
    p.setJointMotorControl2(model,14, p.POSITION_CONTROL, targetPosition=L_hip_roll_value, maxVelocity=2)
    p.setJointMotorControl2(model,15, p.POSITION_CONTROL, targetPosition=L_hip_pitch_value, maxVelocity=2)
    p.setJointMotorControl2(model,16, p.POSITION_CONTROL, targetPosition=L_knee_pitch_value, maxVelocity=2)
    p.setJointMotorControl2(model,17, p.POSITION_CONTROL, targetPosition=L_ankle_pitch_value, maxVelocity=2)
    p.setJointMotorControl2(model,18, p.POSITION_CONTROL, targetPosition=L_ankle_roll_value, maxVelocity=2)
    p.setJointMotorControl2(model,26, p.POSITION_CONTROL, targetPosition=R_hip_yaw_pitch_value, maxVelocity=2)
    p.setJointMotorControl2(model,27, p.POSITION_CONTROL, targetPosition=R_hip_roll_value, maxVelocity=2)
    p.setJointMotorControl2(model,28, p.POSITION_CONTROL, targetPosition=R_hip_pitch_value, maxVelocity=2)
    p.setJointMotorControl2(model,29, p.POSITION_CONTROL, targetPosition=R_knee_pitch_value, maxVelocity=2)
    p.setJointMotorControl2(model,30, p.POSITION_CONTROL, targetPosition=R_ankle_pitch_value, maxVelocity=2)
    p.setJointMotorControl2(model,31, p.POSITION_CONTROL, targetPosition=R_ankle_roll_value, maxVelocity=2)
    p.setJointMotorControl2(model,39, p.POSITION_CONTROL, targetPosition=L_shoulder_pitch_value, maxVelocity=2)
    p.setJointMotorControl2(model,40, p.POSITION_CONTROL, targetPosition=L_shoulder_roll_value, maxVelocity=2)
    p.setJointMotorControl2(model,41, p.POSITION_CONTROL, targetPosition=L_elbow_yaw_value, maxVelocity=2)
    p.setJointMotorControl2(model,42, p.POSITION_CONTROL, targetPosition=L_elbow_roll_value, maxVelocity=2)
    p.setJointMotorControl2(model,43, p.POSITION_CONTROL, targetPosition=L_wrist_yaw_value, maxVelocity=2)
    p.setJointMotorControl2(model,56, p.POSITION_CONTROL, targetPosition=R_shoulder_pitch_value, maxVelocity=2)
    p.setJointMotorControl2(model,57, p.POSITION_CONTROL, targetPosition=R_shoulder_roll_value, maxVelocity=2)
    p.setJointMotorControl2(model,58, p.POSITION_CONTROL, targetPosition=R_elbow_yaw_value, maxVelocity=2)
    p.setJointMotorControl2(model,59, p.POSITION_CONTROL, targetPosition=R_elbow_roll_value, maxVelocity=2)
    p.setJointMotorControl2(model,60, p.POSITION_CONTROL, targetPosition=R_wrist_yaw_value, maxVelocity=2)

    # Guardar en el JSON
    # Coger segundos a los que se quiere la posicion actual y volcarlo a un fichero JSON
    time_to_go = desired_time_value

    if decided != button_prev_value:
        movement_data = {}

        for joint in used_joints:
            joint_info = p.getJointInfo(model, joint)
            joint_name = joint_info[1].decode("utf-8") # Para que sea legible el nombre del joint
            joint_state = p.getJointState(model, joint)
            joint_position = joint_state[0]

            if time_to_go not in movement_data:
                movement_data[time_to_go] = []  # Creamos una lista para cada tiempo

            # Agregar los datos de la articulación al tiempo correspondiente
            movement_data[time_to_go].append({
                "articulacion": joint_name,
                "posicion": joint_position
            })

        # Convertir el diccionario en una lista de tiempos para sacar fotogramas
        fotogram = []
        for tiempo, articulaciones in movement_data.items():
            fotogram.append({
                "tiempo": tiempo,
                "articulaciones": articulaciones
        })

        # Leer datos existentes del archivo JSON, si existe
        if os.path.exists(name):
            with open(name, "r") as json_file:
                existing_data = json.load(json_file)
                print(existing_data)
        else:
            existing_data = []

        # Agregar nuevos datos a los existentes
        existing_data.extend(fotogram)

        # Escribir todos los datos en el archivo JSON
        with open(name, "w") as json_file:
            json.dump(existing_data, json_file, indent=4)

        print("SAVING...")
        time.sleep(0.5)
        print("SAVED, PLEASE INTRODUCE ANOTHER ONE OR EXIT")
        button_prev_value = decided
