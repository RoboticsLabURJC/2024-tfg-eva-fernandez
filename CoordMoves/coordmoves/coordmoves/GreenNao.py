import CoordMovesLib
import time

CoordMovesLib.stand_still()
time.sleep(1)
CoordMovesLib.setArc(1, 0)
time.sleep(1)
CoordMovesLib.grab_box()
time.sleep(1)
CoordMovesLib.setArc(-1, 0)
time.sleep(1)
CoordMovesLib.setArc(0, 1)
time.sleep(1)
CoordMovesLib.setArc(1, 0, 20)
time.sleep(1)
CoordMovesLib.release_box()
print("Caja movida con éxito")

