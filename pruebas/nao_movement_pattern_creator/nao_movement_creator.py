import pybullet as p
import pybullet_data as pd
import time
import json

time_to_go = 0
used_joints = [1,2,13,14,15,16,17,18,26,27,28,29,30,31,39,40,41,42,43,56,57,58,59,60]

# Preparar mundo y modelo
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(1.0, 90.0, -30.0, [0, 0, 0]) # Poner la cámara en la posición deseada
euler_angles = [0,0,0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0,0,0.35]
model = p.loadURDF("Modelo_NAO/nao.urdf", startPosition, startOrientation)

# Anclar a NAO al suelo para que no se mueva
base_link_index = -1
p.createConstraint(model, base_link_index, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], startPosition)

# Slider del tiempo y un slider para indicar que esa es la secuencia que quieres guardar
button = p.addUserDebugParameter("DONE", 0, 1, 0)
button_prev_value = p.readUserDebugParameter(button)

desired_time = p.addUserDebugParameter("Time", 0, 300, 0)

# Preparar sliders para movimiento
head_yaw = p.addUserDebugParameter("HeadYaw", -2.09, 2.09, 0)
head_pitch = p.addUserDebugParameter("HeadPitch", -0.67, 0.51, 0)

# LADO IZQUIERO
L_hip_yaw_pitch = p.addUserDebugParameter("LHipYawPitch", -1.15, 0.74, 0)
L_hip_pitch = p.addUserDebugParameter("LHipPitch", -1.54, 0.48, 0)
L_hip_roll = p.addUserDebugParameter("LHipRoll", -0.38, 0.79, 0)

L_knee_pitch = p.addUserDebugParameter("LKneePitch", -0.09, 2.11, 0)

L_ankle_pitch = p.addUserDebugParameter("LAnklePitch", -1.19, 0.92, 0)
L_ankle_roll = p.addUserDebugParameter("LAnkleRoll", -0.40, 0.77, 0)

L_shoulder_pitch = p.addUserDebugParameter("LShoulderPitch", -5, 5, 0)
L_shoulder_roll = p.addUserDebugParameter("LShoulderRoll", -0.31, 1.33, 0)

L_elbow_yaw = p.addUserDebugParameter("LElbowYaw", -2.09, 2.09, 0)
L_elbow_roll = p.addUserDebugParameter("LElbowRoll", -1.54, -0.03, 0)

L_wrist_yaw = p.addUserDebugParameter("LWristYaw", -1.82, 1.82, 0)

# LADO DERECHO
R_hip_yaw_pitch = p.addUserDebugParameter("RHipYawPitch", -1.15, 0.74, 0)
R_hip_pitch = p.addUserDebugParameter("RHipPitch", -1.54, 0.48, 0)
R_hip_roll = p.addUserDebugParameter("RHipRoll", -0.79, 0.38, 0)

R_knee_pitch = p.addUserDebugParameter("RKneePitch", -0.09, 2.11, 0)

R_ankle_pitch = p.addUserDebugParameter("RAnklePitch", -1.19, 0.93, 0)
R_ankle_roll = p.addUserDebugParameter("RAnkleRoll", -0.77, 0.40, 0)

R_shoulder_pitch = p.addUserDebugParameter("RShoulderPitch", -5, 5, 0)
R_shoulder_roll = p.addUserDebugParameter("RShoulderRoll", -1.33, 0.31, 0)

R_elbow_yaw = p.addUserDebugParameter("RElbowYaw", -2.09, 2.09, 0)
R_elbow_roll = p.addUserDebugParameter("RElbowRoll", -0.03, 1.54, 0)

R_wrist_yaw = p.addUserDebugParameter("RWristYaw", -1.82, 1.82, 0)

# Preparar simulación
while True:
    p.stepSimulation()
    time.sleep(1./240.)

    # Controlar tiempo
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

    p.setJointMotorControl2(model,1, p.POSITION_CONTROL, targetPosition=head_yaw_value)
    p.setJointMotorControl2(model,2, p.POSITION_CONTROL, targetPosition=head_pitch_value)
    p.setJointMotorControl2(model,13, p.POSITION_CONTROL, targetPosition=L_hip_yaw_pitch_value)
    p.setJointMotorControl2(model,14, p.POSITION_CONTROL, targetPosition=L_hip_roll_value)
    p.setJointMotorControl2(model,15, p.POSITION_CONTROL, targetPosition=L_hip_pitch_value)
    p.setJointMotorControl2(model,16, p.POSITION_CONTROL, targetPosition=L_knee_pitch_value)
    p.setJointMotorControl2(model,17, p.POSITION_CONTROL, targetPosition=L_ankle_pitch_value)
    p.setJointMotorControl2(model,18, p.POSITION_CONTROL, targetPosition=L_ankle_roll_value)
    p.setJointMotorControl2(model,26, p.POSITION_CONTROL, targetPosition=R_hip_yaw_pitch_value)
    p.setJointMotorControl2(model,27, p.POSITION_CONTROL, targetPosition=R_hip_roll_value)
    p.setJointMotorControl2(model,28, p.POSITION_CONTROL, targetPosition=R_hip_pitch_value)
    p.setJointMotorControl2(model,29, p.POSITION_CONTROL, targetPosition=R_knee_pitch_value)
    p.setJointMotorControl2(model,30, p.POSITION_CONTROL, targetPosition=R_ankle_pitch_value)
    p.setJointMotorControl2(model,31, p.POSITION_CONTROL, targetPosition=R_ankle_roll_value)
    p.setJointMotorControl2(model,39, p.POSITION_CONTROL, targetPosition=L_shoulder_pitch_value)
    p.setJointMotorControl2(model,40, p.POSITION_CONTROL, targetPosition=L_shoulder_roll_value)
    p.setJointMotorControl2(model,41, p.POSITION_CONTROL, targetPosition=L_elbow_yaw_value)
    p.setJointMotorControl2(model,42, p.POSITION_CONTROL, targetPosition=L_elbow_roll_value)
    p.setJointMotorControl2(model,43, p.POSITION_CONTROL, targetPosition=L_wrist_yaw_value)
    p.setJointMotorControl2(model,56, p.POSITION_CONTROL, targetPosition=R_shoulder_pitch_value)
    p.setJointMotorControl2(model,57, p.POSITION_CONTROL, targetPosition=R_shoulder_roll_value)
    p.setJointMotorControl2(model,58, p.POSITION_CONTROL, targetPosition=R_elbow_yaw_value)
    p.setJointMotorControl2(model,59, p.POSITION_CONTROL, targetPosition=R_elbow_roll_value)
    p.setJointMotorControl2(model,60, p.POSITION_CONTROL, targetPosition=R_wrist_yaw_value)

    # Coger segundos a los que se quiere la posicion actual y volcarlo a un fichero JSON
    time_to_go = desired_time_value

    if decided != button_prev_value and decided == 1:
        movement_data = []

        for joint in used_joints:
            joint_info = p.getJointInfo(model, joint)
            joint_name = joint_info[1].decode("utf-8") # Para que sea legible el nombre del joint
            joint_state = p.getJointState(model, joint)
            joint_position = joint_state[0]

            # Datos para el JSON
            data = {
                "articulacion": joint_name,
                "posicion": joint_position,
                "tiempo": time_to_go
            }

            movement_data.append(data)

            # Escribir todos los datos en un archivo JSON
            with open("movement_pattern.json", "w") as json_file:
                json.dump(movement_data, json_file, indent=4)

        time.sleep(0.5)
        break
    