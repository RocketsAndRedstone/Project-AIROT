#importing needed libraries
import krpc
from time import sleep
from threading import Thread , Event
from CircleQueue import CircleQueue
from PID import PID

def main():
    #connecting to KSP
    conn = krpc.connect(name="Intercept Control")
    crafts = conn.space_center.vessels

    #cycleing through active crafts
    for i in range (len(crafts)):
        #Creating seperate craft variables for future calculations and control schemes 
        if (crafts[i].name == "Crewed Orbital Rendezvous Craft"):
            interceptor = crafts[i]
        elif (crafts[i].name == "Agena Space Station"):         
            targetCraft = crafts[i]
        else:
            continue

if (__name__ == "__main__"):
    main()
    quit()