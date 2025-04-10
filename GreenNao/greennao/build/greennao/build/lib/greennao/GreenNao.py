import GreenNaoLib

orientation = GreenNaoLib.get_face()

if orientation == "face normal":
    GreenNaoLib.stand_still()

elif orientation == "face up":
    GreenNaoLib.wakeup_face_up()
    
elif orientation == "face down":
    GreenNaoLib.wakeup_face_down()

else:
    print("ERROR: Orientación inesperada, tomando posición stand...")
    GreenNaoLib.stand_still()
