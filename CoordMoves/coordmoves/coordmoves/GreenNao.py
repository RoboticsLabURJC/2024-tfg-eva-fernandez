import CoordMovesLib
import time


CoordMovesLib.setArc(0,0)
time.sleep(0.2)
CoordMovesLib.grab_box()
time.sleep(1)
CoordMovesLib.release_box()
print("He terminado!")
