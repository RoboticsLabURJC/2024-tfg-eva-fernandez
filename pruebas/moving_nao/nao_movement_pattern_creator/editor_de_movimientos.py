import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Crear la ventana principal
root = tk.Tk()
root.title("EDITOR DE MOVIMIENTOS PARA NAO")

# Crear campos de entrada y botones
frame_input = tk.Frame(root)
frame_input.pack(padx=10, pady=10)

label_filename = tk.Label(frame_input, text="Nombre del archivo JSON:")
label_filename.pack(side=tk.LEFT)
entry_filename = tk.Entry(frame_input)
entry_filename.pack(side=tk.LEFT)

# Crear sliders
frame_sliders = tk.Frame(root)
frame_sliders.pack(padx=10, pady=10)

slider_amplitude = tk.Scale(frame_sliders, from_=0.1, to=5, orient=tk.HORIZONTAL, label="Amplitud")
slider_amplitude.pack()

slider_period = tk.Scale(frame_sliders, from_=0.1, to=5, orient=tk.HORIZONTAL, label="Periodo")
slider_period.pack()

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

# Función para actualizar la gráfica
def update_graph(event=None):
    # Obtener los valores actuales de los sliders
    amplitude = slider_amplitude.get()
    period = slider_period.get()  # Periodo en segundos (ya no es necesario multiplicarlo por 2pi)
    
    # Generar datos de la simulación
    t = np.linspace(0, 10, 500)  # Tiempo de 0 a 10s con 500 puntos
    y_left = amplitude * np.sin(t * 2 * np.pi / period + np.pi)  # Pierna izquierda con desfase
    y_right = amplitude * np.sin(t * 2 * np.pi / period)  # Pierna derecha

    # Actualizar los datos de la gráfica
    line_left.set_xdata(t)
    line_left.set_ydata(y_left)
    line_right.set_xdata(t)
    line_right.set_ydata(y_right)
    ax.set_xlim(0, max(t))  # El eje X se ajusta según el tiempo
    ax.set_ylim(-amplitude-0.5, amplitude+0.5)  # El eje Y se ajusta según la amplitud

    # Redibujar la gráfica
    canvas.draw()

# Vincular la función de actualización a los sliders
slider_amplitude.config(command=update_graph)
slider_period.config(command=update_graph)

# Crear botón de finalización
button_done = tk.IntVar()
check_done = tk.Checkbutton(root, text="Guardar datos", variable=button_done)
check_done.pack()

# Iniciar el bucle principal
root.mainloop()

