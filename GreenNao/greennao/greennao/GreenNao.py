import GreenNaoLib
import time

orientation = GreenNaoLib.get_face()

while orientation != "face normal":
    if orientation == "face normal":
        GreenNaoLib.stand_still()

    elif orientation == "face up":
        GreenNaoLib.wakeup_face_up()

    elif orientation == "face down":
        GreenNaoLib.wakeup_face_down()

    else:
        print("ERROR: Orientación inesperada, tomando posición estándar...")
        GreenNaoLib.stand_still()

    orientation = GreenNaoLib.get_face()

time.sleep(1)
GreenNaoLib.SetV(2.3)
