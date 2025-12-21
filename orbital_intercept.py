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
    changeOrbitSize(100000 , 150000 , interceptor , conn)
    print(calculateRelativeAngle(interceptor , targetCraft))


#Changes the apopasis and periapsis altitudes while time warping to each burn
def changeOrbitSize(newPeriapsis:float , newApoapsis:float , vessel , conn)->None:
    #Break function for the recursive call
    if ((vessel.orbit.apoapsis_altitude + 1000 > newApoapsis > vessel.orbit.apoapsis_altitude - 1000) and
        (vessel.orbit.periapsis_altitude + 1000 > newPeriapsis > vessel.orbit.periapsis_altitude - 1000)):
        return

    #Time warps to apoapsis or periapsis, whichever is closer
    if (vessel.orbit.time_to_apoapsis < vessel.orbit.time_to_periapsis):
        conn.space_center.warp_to((conn.space_center.ut + vessel.orbit.time_to_apoapsis - 15))

    else:
        conn.space_center.warp_to((conn.space_center.ut + vessel.orbit.time_to_periapsis - 15))

    #calls the apropriate functions to change the apoapsis and periapsis
    if (vessel.orbit.time_to_apoapsis < 20):
        changePeriapsis(vessel , newPeriapsis)

    elif (vessel.orbit.time_to_periapsis < 20):
        changeApoapsis(vessel, newApoapsis)

    #recursive call to ensure propper orbit size
    changeOrbitSize(newPeriapsis, newApoapsis, vessel, conn)

#Changes the apoapsis to the desired height
def changeApoapsis(vessel, newApoapsis:float)->bool:
    #Checks if the apoapsis needs to be raised or lowered
    if (newApoapsis < vessel.orbit.apoapsis_altitude):
        #Sets propper vessel orientation
        vessel.control.sas_mode = vessel.control.sas_mode.retrograde
        sleep(5)
        #Conducts the burn to lower the apoapsis
        while (vessel.orbit.apoapsis_altitude > newApoapsis):
            vessel.control.throttle = 0.5
        vessel.control.throttle = 0
        return True

    elif(newApoapsis > vessel.orbit.apoapsis_altitude):
        #Sets the propper vessel orientation
        vessel.control.sas_mode = vessel.control.sas_mode.prograde
        sleep(5)
        #Conducts burn to raise the apoapsis
        while (vessel.orbit.apoapsis_altitude < newApoapsis):
            vessel.control.throttle = 0.5
        vessel.control.throttle = 0
        return True

    return False

#Changes to periapsis to the desired height
def changePeriapsis(vessel, newPeriapsis:float)->bool:
    #Checks if the periapsis needs to be raised or lowered
    if (newPeriapsis < vessel.orbit.periapsis_altitude):
        #Sets propper vessel orientation
        vessel.control.sas_mode = vessel.control.sas_mode.retrograde
        sleep(5)
        #Conducts burn to lower periapsis
        while (vessel.orbit.periapsis_altitude > newPeriapsis):
            vessel.control.throttle = 0.5
        vessel.control.throttle = 0
        return True

    elif (newPeriapsis > vessel.orbit.periapsis_altitude):
        #Sets propper vessel orientation
        vessel.control.sas_mode = vessel.control.sas_mode.prograde
        sleep(5)
        #Conducts burn to raise periapsis
        while (vessel.orbit.periapsis_altitude < newPeriapsis):
            vessel.control.throttle = 0.5
        vessel.control.throttle = 0
        return True
    
    return False


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