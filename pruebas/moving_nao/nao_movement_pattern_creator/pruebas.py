import tkinter as tk
from tkinter import messagebox
import pybullet as p
import pybullet_data as pd
import numpy as np
import json
import os
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Función para cargar el archivo y crear el archivo JSON si no existe
def load_file():
    file_name = entry_filename.get()
    while not file_name.endswith(".json"):
        file_name = entry_filename.get()
        messagebox.showwarning("Advertencia", "No olvides la extensión .json")
    global name
    name = "/home/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/" + file_name

    # Crear archivo JSON si no existe
    if not os.path.exists(name):
        with open(name, "w") as json_file:
            json.dump([], json_file, indent=4)
    print(f"Archivo {file_name} cargado correctamente.")

# Función para controlar los sliders y la simulación
def start_simulation():
    # Inicializar la simulación de PyBullet
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    planeId = p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(1.0, 89.60, -4.40, [0, 0, 2])
    euler_angles = [0, 0, 0]
    startOrientation = p.getQuaternionFromEuler(euler_angles)
    startPosition = [0, 0, 2]
    model = p.loadURDF("/home/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/Modelo_NAO/nao.urdf", startPosition, startOrientation)
    base_link_index = -1
    p.createConstraint(model, base_link_index, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], startPosition)

    # Obtener los valores de los sliders
    head_yaw_value = slider_head_yaw.get()
    head_pitch_value = slider_head_pitch.get()

    # Control de la simulación (en tu caso, este código debe continuar interactuando con la simulación)
    p.setJointMotorControl2(model, 1, p.POSITION_CONTROL, targetPosition=head_yaw_value)
    p.setJointMotorControl2(model, 2, p.POSITION_CONTROL, targetPosition=head_pitch_value)

    # Actualizar la gráfica con los datos de la simulación
    t = np.linspace(0, 10, 500)  # Tiempo de 0 a 10s con 500 puntos
    amplitude = slider_amplitude.get()
    period = slider_period.get() * 2 * np.pi
    y_left = amplitude * np.sin(t / period + np.pi)
    y_right = amplitude * np.sin(t / period)

    line_left.set_xdata(t)
    line_left.set_ydata(y_left)
    line_right.set_xdata(t)
    line_right.set_ydata(y_right)
    ax.set_xlim(0, max(t))
    ax.set_ylim(-1, 1)
    canvas.draw()

    # Guardar los datos en un archivo JSON
    if button_done.get() == 1:
        save_data_to_json(t, amplitude, period)

    p.stepSimulation()
    time.sleep(1. / 240.)

# Función para guardar los datos en el archivo JSON
def save_data_to_json(t, amplitude, period):
    movement_data = {"time": t.tolist(), "amplitude": amplitude, "period": period}
    if os.path.exists(name):
        with open(name, "r") as json_file:
            existing_data = json.load(json_file)
        existing_data.append(movement_data)
    else:
        existing_data = [movement_data]

    with open(name, "w") as json_file:
        json.dump(existing_data, json_file, indent=4)

# Crear la ventana principal
root = tk.Tk()
root.title("Simulación de Movimiento NAO")

# Crear campos de entrada y botones
frame_input = tk.Frame(root)
frame_input.pack(padx=10, pady=10)

label_filename = tk.Label(frame_input, text="Nombre del archivo JSON:")
label_filename.pack(side=tk.LEFT)
entry_filename = tk.Entry(frame_input)
entry_filename.pack(side=tk.LEFT)

button_load_file = tk.Button(frame_input, text="Cargar archivo", command=load_file)
button_load_file.pack(side=tk.LEFT)

# Crear sliders
frame_sliders = tk.Frame(root)
frame_sliders.pack(padx=10, pady=10)

slider_head_yaw = tk.Scale(frame_sliders, from_=-2.09, to=2.09, orient=tk.HORIZONTAL, label="Head Yaw")
slider_head_yaw.pack()

slider_head_pitch = tk.Scale(frame_sliders, from_=-0.67, to=0.51, orient=tk.HORIZONTAL, label="Head Pitch")
slider_head_pitch.pack()

slider_amplitude = tk.Scale(frame_sliders, from_=0.1, to=1.0, orient=tk.HORIZONTAL, label="Amplitud")
slider_amplitude.pack()

slider_period = tk.Scale(frame_sliders, from_=0.01, to=0.5, orient=tk.HORIZONTAL, label="Periodo")
slider_period.pack()

# Crear botón de inicio de simulación
button_start = tk.Button(root, text="Iniciar Simulación", command=start_simulation)
button_start.pack(pady=10)

# Crear área de la gráfica
fig, ax = plt.subplots()
line_left, = ax.plot([], [], 'b-', label="Pierna Izquierda")
line_right, = ax.plot([], [], 'r-', label="Pierna Derecha")
ax.set_xlabel("Tiempo (s)")
ax.set_ylabel("Desplazamiento")
ax.set_title("Movimiento de las piernas")
ax.axhline(0, color="gray", linestyle="--")
ax.legend()
ax.grid()

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()

# Crear botón de finalización
button_done = tk.IntVar()
check_done = tk.Checkbutton(root, text="Guardar datos", variable=button_done)
check_done.pack()

# Iniciar el bucle principal
root.mainloop()

