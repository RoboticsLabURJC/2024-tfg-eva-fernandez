import CoordMovesLib
import time

time.sleep(5)

CoordMovesLib.grab_box()

time.sleep(3)

for i in range(3):
  CoordMovesLib.turn("L", 60)

CoordMovesLib.turn("L", 40)

CoordMovesLib.setArc(1, 0)

time.sleep(3)

CoordMovesLib.release_box()
print("Terminado")
