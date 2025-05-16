import CoordMovesLib
import time

CoordMovesLib.stand_still()
time.sleep(1)
CoordMovesLib.setArc(1, 0, 8)
time.sleep(1)
CoordMovesLib.grab_box()
time.sleep(1)
CoordMovesLib.setArc(-1, 0, 4)
time.sleep(1)
CoordMovesLib.setArc(0, 1, 8)
time.sleep(1)
CoordMovesLib.setArc(1, 0, 18)
time.sleep(1)
CoordMovesLib.release_box()
print("Caja movida con éxito")

