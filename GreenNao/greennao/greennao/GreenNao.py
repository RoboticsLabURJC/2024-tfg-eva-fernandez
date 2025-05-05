import GreenNaoLib
import time

GreenNaoLib.Interpreter("grab.json") # Primero giramos en el sitio a velocidad normal
time.sleep(1)
#GreenNaoLib.stand_still()
GreenNaoLib.setArc(1, 0, 20) # Después avanzamos recto a velocidad máxima
# time.sleep(1)
# GreenNaoLib.setArc(0.5, 0.5, 20) # A conticuación avanzamos en arco a mitad de velocidad
# time.sleep(1)
# GreenNaoLib.setArc(0, 0, 20) # Por último, paramos
print("He terminado!")