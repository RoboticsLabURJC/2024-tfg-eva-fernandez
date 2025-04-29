import GreenNaoLib

orientation = GreenNaoLib.get_face() # Gracias a lecturas de IMU, sabemos si hemos caído y de que forma

if orientation == "face normal":
    GreenNaoLib.stand_still()

elif orientation == "face up":
    GreenNaoLib.wakeup_face_up()

elif orientation == "face down":
    GreenNaoLib.wakeup_face_down()

else:
    print("ERROR: No puedo levantarme")
    sys.exit(1)












































# orientation = GreenNaoLib.get_face()

# while orientation != "face normal":
#     if orientation == "face normal":
#         GreenNaoLib.stand_still()

#     elif orientation == "face up":
#         GreenNaoLib.wakeup_face_up()

#     elif orientation == "face down":
#         GreenNaoLib.wakeup_face_down()

#     else:
#         print("ERROR: Orientación inesperada, tomando posición estándar...")
#         GreenNaoLib.stand_still()

#     time.sleep(0.05)
#     orientation = GreenNaoLib.get_face()
    

# time.sleep(1.5)
# GreenNaoLib.SetV(1.5)
