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

    #print(calculateRelativeAngle(interceptor , targetCraft))    
    changeOrbitSize(100000 , 150000 , interceptor , conn)
    matchInclination(interceptor , targetCraft , conn)
    #print(calculateRelativeAngle(interceptor , targetCraft))


#Changes the apoapasis and periapsis altitudes while time warping to each burn
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
def changeApoapsis(vessel, newApoapsis:float)->None:
    #Checks if the apoapsis needs to be raised or lowered
    if (newApoapsis < vessel.orbit.apoapsis_altitude):
        #Sets propper vessel orientation
        vessel.control.sas_mode = vessel.control.sas_mode.retrograde
        sleep(5)
        #Conducts the burn to lower the apoapsis
        while (vessel.orbit.apoapsis_altitude > newApoapsis):
            vessel.control.throttle = 0.5
        vessel.control.throttle = 0

    elif(newApoapsis > vessel.orbit.apoapsis_altitude):
        #Sets the propper vessel orientation
        vessel.control.sas_mode = vessel.control.sas_mode.prograde
        sleep(5)
        #Conducts burn to raise the apoapsis
        while (vessel.orbit.apoapsis_altitude < newApoapsis):
            vessel.control.throttle = 0.5
        vessel.control.throttle = 0

#Changes to periapsis to the desired height
def changePeriapsis(vessel, newPeriapsis:float)->None:
    #Checks if the periapsis needs to be raised or lowered
    if (newPeriapsis < vessel.orbit.periapsis_altitude):
        #Sets propper vessel orientation
        vessel.control.sas_mode = vessel.control.sas_mode.retrograde
        sleep(5)
        #Conducts burn to lower periapsis
        while (vessel.orbit.periapsis_altitude > newPeriapsis):
            vessel.control.throttle = 0.5
        vessel.control.throttle = 0

    elif (newPeriapsis > vessel.orbit.periapsis_altitude):
        #Sets propper vessel orientation
        vessel.control.sas_mode = vessel.control.sas_mode.prograde
        sleep(5)
        #Conducts burn to raise periapsis
        while (vessel.orbit.periapsis_altitude < newPeriapsis):
            vessel.control.throttle = 0.5
        vessel.control.throttle = 0


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

#Changing the inclination to be within ~0.0001 radian (0.005 degrees)
def matchInclination(interceptor, targetCraft , conn) -> None:
    #Time warps to the next node
    while not (0.00955 > interceptor.flight().latitude > -0.00955):
        conn.space_center.rails_warp_factor = 4
    conn.space_center.rails_warp_factor = 0

    #Determines if the ascending or descending node is closser by checking latitude and orients acordingly
    if (interceptor.flight().latitude > 0):
        interceptor.control.sas_mode = interceptor.control.sas_mode.normal 
    else:
        interceptor.control.sas_mode = interceptor.control.sas_mode.anti_normal

    #waiting for the vessel to orient itself
    sleep(5)

    #conducts the plane change burn
    while (math.fabs(interceptor.orbit.inclination - targetCraft.orbit.inclination) > 0.0001):
        interceptor.control.throttle = 0.25
    interceptor.control.throttle = 0

#Calculates the seperation between the vessels
def distance(interceptor, targetCraft) -> float:
    pass
    kerbinRadius = 600000
    #vertical speration
    seperation_x = math.fabs((interceptor.orbit.radius - kerbinRadius) - (targetCraft.orbit.radius - kerbinRadius))

if (__name__ == "__main__"):
    main()
    quit()