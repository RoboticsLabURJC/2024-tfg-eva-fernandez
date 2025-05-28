import CoordMovesLib
import time

CoordMovesLib.stand_still()

CoordMovesLib.setArc(1, 0)

CoordMovesLib.grab_box()

for i in range(3):
  CoordMovesLib.turn("L", 60)

CoordMovesLib.turn("L", 40)

CoordMovesLib.setArc(1, 0, 20)

CoordMovesLib.release_box()
print("Terminado")
