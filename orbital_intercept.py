#importing needed libraries
import krpc
import math
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

    print(calculateRelativeAngle(interceptor , targetCraft))

def calculateRelativeAngle(interceptor, targetCraft) -> float:
    #kerbin radius for degree to meter calculation
    kerbinRadius = 600000

    #getting each vessel's longitude
    targetLongitude = targetCraft.flight().longitude
    interceptorLongitude = interceptor.flight().longitude

    #converting the longitude to a standard distance
    targetLongitude = math.radians(targetLongitude) * kerbinRadius
    interceptorLongitude = math.radians(interceptorLongitude) * kerbinRadius

    #angle calculations
    targetAngle = math.degrees(math.atan(targetCraft.flight().surface_altitude / targetLongitude))
    interceptAngle = math.degrees(math.atan(interceptor.flight().surface_altitude / interceptorLongitude))

    relativeAngle = interceptAngle + targetAngle 

    return relativeAngle

if (__name__ == "__main__"):
    main()
    quit()