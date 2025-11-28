import coind4 
import initialisation

ldr = coind4.CoinD4() #lidar initialise
ldr.start()

while True:
    pose = initialisation.initial_pose(ldr)
    print("Initial pose:", pose)
