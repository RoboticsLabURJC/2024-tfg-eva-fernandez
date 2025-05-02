import GreenNaoLib
import time

GreenNaoLib.setArc(0, 1, 20) # Primero giramos en el sitio a velocidad normal
time.sleep(1)
GreenNaoLib.setArc(1, 0, 20) # Después avanzamos recto a velocidad máxima
time.sleep(1)
GreenNaoLib.setArc(0.5, 0.5, 20) # A conticuación avanzamos en arco a mitad de velocidad
time.sleep(1)
GreenNaoLib.setArc(0, 0, 20) # Por último, paramos
print("He terminado!")