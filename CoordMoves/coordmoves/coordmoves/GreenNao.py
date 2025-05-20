import CoordMovesLib
import time


CoordMovesLib.setArc(1, 0)
time.sleep(3)
CoordMovesLib.grab_box()
time.sleep(3)
CoordMovesLib.turn("L", 60)
time.sleep(3)
CoordMovesLib.setArc(1, 0)
time.sleep(3)
CoordMovesLib.turn("R", 60)
time.sleep(3)
CoordMovesLib.release_box()
print("Caja movida con éxito")

